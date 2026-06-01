// hyumm_vive_xddp_node -- the low-level nrt node.
//
// Reads the RAW libsurvive tracker topics directly (/vive/<name>/pose|twist),
// runs the vive localization IN-PROCESS (hyumm_vive::ViveLocalizer: anchor on
// the fixed tracker -> vive_world, + mobile-base center pose/twist), broadcasts
// the result as TF (for RViz / high-level), and is the SOLE XDDP bridge to the
// xenomai RT controller:
//     nrt -> rt : odom    (base pose + twist)   on /dev/rtp<odom_port>    (rx_odom_task)
//     nrt -> rt : cmd_vel (forwarded /cmd_vel)  on /dev/rtp<cmdvel_port>  (rx_cmdvel_task)
//     rt -> nrt : full robot state RobotInfo    on /dev/rtp<status_port>  (tx_robot_state_task)
//
// The rt->nrt RobotInfo carries act.q + nom.q for all MM_DOF joints. From it we:
//   * publish /joint_states          (actual arm, act.q[3..8])  -> rsp_actual
//   * publish /nominal/joint_states  (nominal arm, nom.q[3..8]) -> rsp_nominal
//   * broadcast vive_world -> nominal/base_link (nominal base, nom.q[0..2])
// so RViz shows the actual robot (base from vive, arm from RT act) AND the green
// nominal robot (base + arm from RT nom) moving together.
//
// Each XDDP endpoint is an XddpLink with a watchdog (opens /dev/rtpN only while
// its owning RT task is alive), so either side restarts without cycling the
// other. The node NEVER exits on a missing RT side.

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include "hyumm_vive/vive_localizer.h"
#include "xddp_ros.h"   // XddpLink (self-healing /dev/rtpN) + packet:: schema

namespace {
// Mobile-manipulator DOF layout (must match RTECAT_MobileManipulator: MM_DOF_NUM
// = MOBILE_DOF_NUM(3) + NRMK_DRIVE_NUM(6)). q[0..2] = base x/y/yaw, q[3..8] = arm.
constexpr int MM_DOF     = 9;
constexpr int ARM_OFFSET = 3;
constexpr int ARM_DOF    = 6;
}  // namespace

class ViveXddpNode {
public:
  ViveXddpNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    hyumm_vive::ViveLocalizer::Config c;
    pnh.param<std::string>("reference", c.reference, c.reference);
    pnh.param<std::string>("rear", c.rear, c.rear);
    pnh.param<std::string>("front", c.front, c.front);
    pnh.param<std::string>("world_frame", c.world_frame, c.world_frame);
    pnh.param<std::string>("source_frame", c.source_frame, c.source_frame);
    pnh.param<std::string>("base_frame", c.base_frame, c.base_frame);
    std::vector<double> ap;
    if (pnh.getParam("anchor_position", ap) && ap.size() == 3)
      c.anchor_pos = tf::Vector3(ap[0], ap[1], ap[2]);
    std::vector<double> ao;
    if (pnh.getParam("anchor_orientation", ao)) {
      if (ao.size() == 4) {
        tf::Quaternion q(ao[0], ao[1], ao[2], ao[3]);
        if (q.length2() > 1e-9) c.anchor_quat = q.normalized();
      } else if (ao.size() == 3) {
        tf::Quaternion q; q.setRPY(ao[0], ao[1], ao[2]); c.anchor_quat = q;
      }
    }
    pnh.param("settle_samples", c.settle_samples, c.settle_samples);
    pnh.param("warmup_samples", c.warmup_samples, c.warmup_samples);
    pnh.param("offset_rear_x", c.orx, c.orx);
    pnh.param("offset_rear_y", c.ory, c.ory);
    pnh.param("offset_front_x", c.ofx, c.ofx);
    pnh.param("offset_front_y", c.ofy, c.ofy);
    pnh.param("occlusion_timeout", c.occlusion_timeout, c.occlusion_timeout);
    loc_.configure(c);
    cfg_ = c;

    pnh.param<std::string>("input_ns", input_ns_, "/vive");
    pnh.param("publish_tf", publish_tf_, true);
    pnh.param("send_xddp", send_xddp_, false);
    pnh.param("xddp_odom_port", odom_port_, static_cast<int>(XDDP_PORT_CMD_INFO_2));
    pnh.param("xddp_cmdvel_port", cmd_port_, static_cast<int>(XDDP_PORT_CMD_INFO_1));
    // rt->nrt full robot state (RobotInfo: act.q + nom.q) -> actual + nominal
    // joint_states + the nominal base TF (vive_world -> nominal_base_frame).
    pnh.param("recv_status", recv_status_, false);
    pnh.param("xddp_status_port", status_port_, static_cast<int>(XDDP_PORT_STATE_INFO_1));
    pnh.param<std::string>("nominal_base_frame", nominal_base_frame_, "nominal/base_link");
    // Ground height for base_link in vive_world. The trackers sit ~0.8 m up but
    // the URDF base_link is the chassis bottom, so drop it to the floor (default
    // 0). x/y/yaw still come from vive; only z is overridden.
    pnh.param("base_z", base_z_, 0.0);

    // RT task names the watchdog greps for in /proc/xenomai/sched/threads (the
    // task that BINDS each port; must match rt_task_create in xddp_bridge.cpp).
    std::string rt_odom, rt_cmdvel, rt_status;
    pnh.param<std::string>("rt_task_odom",   rt_odom,   "rx_odom_task");
    pnh.param<std::string>("rt_task_cmdvel", rt_cmdvel, "rx_cmdvel_task");
    pnh.param<std::string>("rt_task_status", rt_status, "tx_robot_state_task");
    pnh.param("watchdog_period", watchdog_period_, 0.5);

    odom_link_.configure("odom",            odom_port_,   rt_odom,   send_xddp_);
    cmd_link_.configure("cmd_vel",          cmd_port_,    rt_cmdvel, send_xddp_);
    status_link_.configure("status(state rx)", status_port_, rt_status, recv_status_);

    // arm joint names (Indy7 revolute joints in hyumm_scan.urdf)
    for (int i = 0; i < ARM_DOF; ++i) arm_names_.push_back("joint" + std::to_string(i));
    for (int i = 0; i < ARM_DOF; ++i) { act_arm_[i] = 0.0; nom_arm_[i] = 0.0; }

    sub_fixed_ = nh.subscribe<geometry_msgs::PoseStamped>(
        input_ns_ + "/" + c.reference + "/pose", 10,
        boost::bind(&ViveXddpNode::poseCb, this, _1, c.reference));
    sub_rear_ = nh.subscribe<geometry_msgs::PoseStamped>(
        input_ns_ + "/" + c.rear + "/pose", 10,
        boost::bind(&ViveXddpNode::poseCb, this, _1, c.rear));
    sub_front_ = nh.subscribe<geometry_msgs::PoseStamped>(
        input_ns_ + "/" + c.front + "/pose", 10,
        boost::bind(&ViveXddpNode::poseCb, this, _1, c.front));
    sub_rear_tw_ = nh.subscribe<geometry_msgs::TwistStamped>(
        input_ns_ + "/" + c.rear + "/twist", 10,
        boost::bind(&ViveXddpNode::twistCb, this, _1, c.rear));
    sub_front_tw_ = nh.subscribe<geometry_msgs::TwistStamped>(
        input_ns_ + "/" + c.front + "/twist", 10,
        boost::bind(&ViveXddpNode::twistCb, this, _1, c.front));
    sub_cmd_ = nh.subscribe("/cmd_vel", 1, &ViveXddpNode::cmdCb, this);
    base_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/vive_world/" + c.base_frame + "/pose", 10);
    // Replaces joint_state_publisher: actual arm from RT act.q, nominal from nom.q.
    act_joints_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    nom_joints_pub_ = nh.advertise<sensor_msgs::JointState>("/nominal/joint_states", 10);

    // Watchdog: probe once now then on a wall-clock timer.
    watchdogTick();
    watchdog_ = nh.createWallTimer(ros::WallDuration(watchdog_period_),
                                   &ViveXddpNode::watchdogCb, this);

    ROS_INFO("hyumm_vive_xddp_node: localize from %s, anchor on '%s' -> %s; "
             "send_xddp=%s recv_status=%s (XDDP links auto-(re)connect to RT)",
             input_ns_.c_str(), c.reference.c_str(), c.world_frame.c_str(),
             send_xddp_ ? "true" : "false", recv_status_ ? "true" : "false");
  }

  void watchdogCb(const ros::WallTimerEvent&) { watchdogTick(); }
  void watchdogTick() {
    odom_link_.poll();
    cmd_link_.poll();
    status_link_.poll();
  }

  // Drain the rt->nrt RobotInfo and keep the latest act/nom joints + nominal base.
  void readRobotState() {
    packet::RobotInfo<MM_DOF> rx;
    bool got = false;
    while (status_link_.tryRead(rx)) { last_ = rx; got = true; }
    if (!got) return;
    have_state_ = true;
    nominal_x_ = last_.nom.jointState.position[0];
    nominal_y_ = last_.nom.jointState.position[1];
    nominal_quat_.setRPY(0.0, 0.0, last_.nom.jointState.position[2]);
    for (int i = 0; i < ARM_DOF; ++i) {
      act_arm_[i] = last_.act.jointState.position[ARM_OFFSET + i];
      nom_arm_[i] = last_.nom.jointState.position[ARM_OFFSET + i];
    }
  }

  // Publish actual + nominal arm joint_states (replaces joint_state_publisher).
  // Always publishes (zeros until the first RT packet) so both robot_state_
  // publishers always have joint data and the arm frames exist.
  void publishJoints(const ros::Time& now) {
    sensor_msgs::JointState ja;
    ja.header.stamp = now;
    ja.name.assign(arm_names_.begin(), arm_names_.end());
    ja.position.assign(act_arm_, act_arm_ + ARM_DOF);
    act_joints_pub_.publish(ja);

    sensor_msgs::JointState jn;
    jn.header.stamp = now;
    jn.name.assign(arm_names_.begin(), arm_names_.end());
    jn.position.assign(nom_arm_, nom_arm_ + ARM_DOF);
    nom_joints_pub_.publish(jn);
  }

  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& m, const std::string& name) {
    loc_.setPose(name, m->pose, ros::Time::now());
  }
  void twistCb(const geometry_msgs::TwistStamped::ConstPtr& m, const std::string& name) {
    loc_.setTwist(name, m->twist);
  }
  void cmdCb(const geometry_msgs::Twist::ConstPtr& m) {
    if (!send_xddp_) return;
    packet::Twist t;
    std::memset(&t, 0, sizeof(t));
    t.linear.x = m->linear.x; t.linear.y = m->linear.y; t.linear.z = m->linear.z;
    t.angular.x = m->angular.x; t.angular.y = m->angular.y; t.angular.z = m->angular.z;
    cmd_link_.tryWrite(t);   // no-op while the RT cmd_vel task is down
  }

  void spin() {
    ros::Time now = ros::Time::now();

    // rt->nrt robot state: drain, then publish actual + nominal joint_states.
    if (recv_status_) readRobotState();
    publishJoints(now);   // always (zeros until first RT packet) so arms render

    if (!loc_.locked()) {
      if (loc_.tryLock())
        ROS_INFO("vive_world anchor LOCKED on '%s' (%zu samples).",
                 cfg_.reference.c_str(), loc_.refSamples());
      else
        ROS_WARN_THROTTLE(2.0, "anchor not locked: %zu/%d reference samples from "
                          "'%s' -- is the fixed tracker tracking?",
                          loc_.refSamples(), cfg_.settle_samples, cfg_.reference.c_str());
      return;  // publish nothing else until vive_world is defined
    }

    if (publish_tf_)
      br_.sendTransform(tf::StampedTransform(loc_.anchorTF(), now,
                                             cfg_.world_frame, cfg_.source_frame));

    // NOMINAL base (rt->nrt): vive_world -> nominal_base_frame at nom x/y/yaw,
    // kept at the actual base height for an overlaid comparison.
    if (recv_status_ && have_state_ && publish_tf_) {
      tf::Transform Tn(nominal_quat_, tf::Vector3(nominal_x_, nominal_y_, base_z_));
      br_.sendTransform(tf::StampedTransform(Tn, now, cfg_.world_frame, nominal_base_frame_));
    }

    tf::Transform T_base;
    geometry_msgs::Twist bt;
    double cyaw;
    if (!loc_.mobileBase(now, T_base, bt, cyaw)) return;  // base occluded
    // Drop base_link to the ground plane (trackers sit higher); x/y/yaw from vive.
    { tf::Vector3 bo = T_base.getOrigin(); bo.setZ(base_z_); T_base.setOrigin(bo); }

    if (publish_tf_)
      br_.sendTransform(tf::StampedTransform(T_base, now, cfg_.world_frame, cfg_.base_frame));

    geometry_msgs::PoseStamped bp;
    bp.header.stamp = now;
    bp.header.frame_id = cfg_.world_frame;
    tf::poseTFToMsg(T_base, bp.pose);
    base_pub_.publish(bp);

    if (send_xddp_) {
      packet::Odometry tx;
      std::memset(&tx, 0, sizeof(tx));
      tx.pose.position.x = T_base.getOrigin().x();
      tx.pose.position.y = T_base.getOrigin().y();
      tx.pose.position.z = cyaw;                 // continuous (multi-turn) yaw
      tx.pose.orientation.x = 0.0;
      tx.pose.orientation.y = 0.0;
      tx.pose.orientation.z = std::sin(cyaw / 2.0);
      tx.pose.orientation.w = std::cos(cyaw / 2.0);
      tx.twist.linear.x = bt.linear.x;
      tx.twist.linear.y = bt.linear.y;
      tx.twist.linear.z = 0.0;
      tx.twist.angular.x = 0.0;
      tx.twist.angular.y = 0.0;
      tx.twist.angular.z = bt.angular.z;
      odom_link_.tryWrite(tx);   // no-op while the RT odom task is down
    }
  }

private:
  hyumm_vive::ViveLocalizer loc_;
  hyumm_vive::ViveLocalizer::Config cfg_;
  std::string input_ns_, nominal_base_frame_;
  bool publish_tf_ = true, send_xddp_ = false, recv_status_ = false, have_state_ = false;
  int odom_port_ = 0, cmd_port_ = 0, status_port_ = 0;
  double watchdog_period_ = 0.5;
  double base_z_ = 0.0, nominal_x_ = 0.0, nominal_y_ = 0.0;
  tf::Quaternion nominal_quat_ = tf::Quaternion(0, 0, 0, 1);

  packet::RobotInfo<MM_DOF> last_{};
  std::vector<std::string> arm_names_;
  double act_arm_[ARM_DOF], nom_arm_[ARM_DOF];

  XddpLink odom_link_, cmd_link_, status_link_;   // self-healing XDDP endpoints

  ros::Subscriber sub_fixed_, sub_rear_, sub_front_, sub_rear_tw_, sub_front_tw_, sub_cmd_;
  ros::Publisher base_pub_, act_joints_pub_, nom_joints_pub_;
  ros::WallTimer watchdog_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hyumm_vive_xddp_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ViveXddpNode node(nh, pnh);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    node.spin();
    rate.sleep();
  }
  return 0;
}
