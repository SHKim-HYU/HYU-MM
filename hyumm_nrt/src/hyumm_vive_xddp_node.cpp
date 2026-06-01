// hyumm_vive_xddp_node -- the low-level nrt node. ROS2 port.
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
//
// Each XDDP endpoint is an XddpLink with a watchdog (opens /dev/rtpN only while
// its owning RT task is alive), so either side restarts without cycling the
// other. The node NEVER exits on a missing RT side.

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hyumm_vive/vive_localizer.h"
#include "xddp_ros.h"   // XddpLink (self-healing /dev/rtpN) + packet:: schema

namespace {
// Mobile-manipulator DOF layout (must match RTECAT_MobileManipulator: MM_DOF_NUM
// = MOBILE_DOF_NUM(3) + NRMK_DRIVE_NUM(6)). q[0..2] = base x/y/yaw, q[3..8] = arm.
constexpr int MM_DOF     = 9;
constexpr int ARM_OFFSET = 3;
constexpr int ARM_DOF    = 6;

// Fill a TransformStamped from a tf2::Transform.
geometry_msgs::msg::TransformStamped makeTf(const tf2::Transform& T,
                                            const rclcpp::Time& stamp,
                                            const std::string& parent,
                                            const std::string& child) {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = stamp;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  const tf2::Vector3 o = T.getOrigin();
  const tf2::Quaternion q = T.getRotation();
  ts.transform.translation.x = o.x();
  ts.transform.translation.y = o.y();
  ts.transform.translation.z = o.z();
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  return ts;
}
}  // namespace

class ViveXddpNode : public rclcpp::Node {
public:
  ViveXddpNode() : rclcpp::Node("hyumm_vive_xddp_node") {
    hyumm_vive::ViveLocalizer::Config c;
    c.reference = declare_parameter<std::string>("reference", c.reference);
    c.rear = declare_parameter<std::string>("rear", c.rear);
    c.front = declare_parameter<std::string>("front", c.front);
    c.world_frame = declare_parameter<std::string>("world_frame", c.world_frame);
    c.source_frame = declare_parameter<std::string>("source_frame", c.source_frame);
    c.base_frame = declare_parameter<std::string>("base_frame", c.base_frame);

    auto ap = declare_parameter<std::vector<double>>("anchor_position",
                  {c.anchor_pos.x(), c.anchor_pos.y(), c.anchor_pos.z()});
    if (ap.size() == 3) c.anchor_pos = tf2::Vector3(ap[0], ap[1], ap[2]);
    auto ao = declare_parameter<std::vector<double>>("anchor_orientation",
                  {0.0, 0.0, 0.0, 1.0});
    if (ao.size() == 4) {
      tf2::Quaternion q(ao[0], ao[1], ao[2], ao[3]);
      if (q.length2() > 1e-9) c.anchor_quat = q.normalized();
    } else if (ao.size() == 3) {
      tf2::Quaternion q; q.setRPY(ao[0], ao[1], ao[2]); c.anchor_quat = q;
    }
    c.settle_samples = declare_parameter<int>("settle_samples", c.settle_samples);
    c.warmup_samples = declare_parameter<int>("warmup_samples", c.warmup_samples);
    c.orx = declare_parameter<double>("offset_rear_x", c.orx);
    c.ory = declare_parameter<double>("offset_rear_y", c.ory);
    c.ofx = declare_parameter<double>("offset_front_x", c.ofx);
    c.ofy = declare_parameter<double>("offset_front_y", c.ofy);
    c.occlusion_timeout = declare_parameter<double>("occlusion_timeout", c.occlusion_timeout);
    loc_.configure(c);
    cfg_ = c;

    input_ns_ = declare_parameter<std::string>("input_ns", "/vive");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    send_xddp_ = declare_parameter<bool>("send_xddp", false);
    odom_port_ = declare_parameter<int>("xddp_odom_port", static_cast<int>(XDDP_PORT_CMD_INFO_2));
    cmd_port_ = declare_parameter<int>("xddp_cmdvel_port", static_cast<int>(XDDP_PORT_CMD_INFO_1));
    recv_status_ = declare_parameter<bool>("recv_status", false);
    status_port_ = declare_parameter<int>("xddp_status_port", static_cast<int>(XDDP_PORT_STATE_INFO_1));
    nominal_base_frame_ = declare_parameter<std::string>("nominal_base_frame", "nominal/base_link");
    base_z_ = declare_parameter<double>("base_z", 0.0);

    std::string rt_odom = declare_parameter<std::string>("rt_task_odom", "rx_odom_task");
    std::string rt_cmdvel = declare_parameter<std::string>("rt_task_cmdvel", "rx_cmdvel_task");
    std::string rt_status = declare_parameter<std::string>("rt_task_status", "tx_robot_state_task");
    watchdog_period_ = declare_parameter<double>("watchdog_period", 0.5);

    odom_link_.configure("odom", odom_port_, rt_odom, send_xddp_);
    cmd_link_.configure("cmd_vel", cmd_port_, rt_cmdvel, send_xddp_);
    status_link_.configure("status(state rx)", status_port_, rt_status, recv_status_);
    odom_link_.setLogger(get_logger());
    cmd_link_.setLogger(get_logger());
    status_link_.setLogger(get_logger());

    // arm joint names (Indy7 revolute joints in hyumm_scan.urdf)
    for (int i = 0; i < ARM_DOF; ++i) arm_names_.push_back("joint" + std::to_string(i));
    for (int i = 0; i < ARM_DOF; ++i) { act_arm_[i] = 0.0; nom_arm_[i] = 0.0; }

    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    using std::placeholders::_1;
    sub_fixed_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input_ns_ + "/" + c.reference + "/pose", 10,
        [this, name = c.reference](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          loc_.setPose(name, m->pose, now().seconds());
        });
    sub_rear_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input_ns_ + "/" + c.rear + "/pose", 10,
        [this, name = c.rear](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          loc_.setPose(name, m->pose, now().seconds());
        });
    sub_front_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input_ns_ + "/" + c.front + "/pose", 10,
        [this, name = c.front](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          loc_.setPose(name, m->pose, now().seconds());
        });
    sub_rear_tw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        input_ns_ + "/" + c.rear + "/twist", 10,
        [this, name = c.rear](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          loc_.setTwist(name, m->twist);
        });
    sub_front_tw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        input_ns_ + "/" + c.front + "/twist", 10,
        [this, name = c.front](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          loc_.setTwist(name, m->twist);
        });
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        std::bind(&ViveXddpNode::cmdCb, this, _1));

    base_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/vive_world/" + c.base_frame + "/pose", 10);
    act_joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    nom_joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("/nominal/joint_states", 10);

    // Watchdog: probe once now then on a wall-clock timer.
    watchdogTick();
    watchdog_ = create_wall_timer(
        std::chrono::duration<double>(watchdog_period_),
        std::bind(&ViveXddpNode::watchdogTick, this));

    RCLCPP_INFO(get_logger(),
        "hyumm_vive_xddp_node: localize from %s, anchor on '%s' -> %s; "
        "send_xddp=%s recv_status=%s (XDDP links auto-(re)connect to RT)",
        input_ns_.c_str(), c.reference.c_str(), c.world_frame.c_str(),
        send_xddp_ ? "true" : "false", recv_status_ ? "true" : "false");
  }

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
  void publishJoints(const rclcpp::Time& stamp) {
    sensor_msgs::msg::JointState ja;
    ja.header.stamp = stamp;
    ja.name.assign(arm_names_.begin(), arm_names_.end());
    ja.position.assign(act_arm_, act_arm_ + ARM_DOF);
    act_joints_pub_->publish(ja);

    sensor_msgs::msg::JointState jn;
    jn.header.stamp = stamp;
    jn.name.assign(arm_names_.begin(), arm_names_.end());
    jn.position.assign(nom_arm_, nom_arm_ + ARM_DOF);
    nom_joints_pub_->publish(jn);
  }

  void cmdCb(geometry_msgs::msg::Twist::ConstSharedPtr m) {
    if (!send_xddp_) return;
    packet::Twist t;
    std::memset(&t, 0, sizeof(t));
    t.linear.x = m->linear.x; t.linear.y = m->linear.y; t.linear.z = m->linear.z;
    t.angular.x = m->angular.x; t.angular.y = m->angular.y; t.angular.z = m->angular.z;
    cmd_link_.tryWrite(t);   // no-op while the RT cmd_vel task is down
  }

  void step() {
    rclcpp::Time stamp = now();

    // rt->nrt robot state: drain, then publish actual + nominal joint_states.
    if (recv_status_) readRobotState();
    publishJoints(stamp);   // always (zeros until first RT packet) so arms render

    if (!loc_.locked()) {
      if (loc_.tryLock())
        RCLCPP_INFO(get_logger(), "vive_world anchor LOCKED on '%s' (%zu samples).",
                    cfg_.reference.c_str(), loc_.refSamples());
      else
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "anchor not locked: %zu/%d reference samples from "
                             "'%s' -- is the fixed tracker tracking?",
                             loc_.refSamples(), cfg_.settle_samples,
                             cfg_.reference.c_str());
      return;  // publish nothing else until vive_world is defined
    }

    if (publish_tf_)
      br_->sendTransform(makeTf(loc_.anchorTF(), stamp,
                                cfg_.world_frame, cfg_.source_frame));

    // NOMINAL base (rt->nrt): vive_world -> nominal_base_frame at nom x/y/yaw.
    if (recv_status_ && have_state_ && publish_tf_) {
      tf2::Transform Tn(nominal_quat_, tf2::Vector3(nominal_x_, nominal_y_, base_z_));
      br_->sendTransform(makeTf(Tn, stamp, cfg_.world_frame, nominal_base_frame_));
    }

    tf2::Transform T_base;
    geometry_msgs::msg::Twist bt;
    double cyaw;
    if (!loc_.mobileBase(stamp.seconds(), T_base, bt, cyaw)) return;  // base occluded
    // Drop base_link to the ground plane (trackers sit higher); x/y/yaw from vive.
    { tf2::Vector3 bo = T_base.getOrigin(); bo.setZ(base_z_); T_base.setOrigin(bo); }

    if (publish_tf_)
      br_->sendTransform(makeTf(T_base, stamp, cfg_.world_frame, cfg_.base_frame));

    geometry_msgs::msg::PoseStamped bp;
    bp.header.stamp = stamp;
    bp.header.frame_id = cfg_.world_frame;
    const tf2::Vector3 o = T_base.getOrigin();
    const tf2::Quaternion q = T_base.getRotation();
    bp.pose.position.x = o.x(); bp.pose.position.y = o.y(); bp.pose.position.z = o.z();
    bp.pose.orientation.x = q.x(); bp.pose.orientation.y = q.y();
    bp.pose.orientation.z = q.z(); bp.pose.orientation.w = q.w();
    base_pub_->publish(bp);

    if (send_xddp_) {
      packet::Odometry tx;
      std::memset(&tx, 0, sizeof(tx));
      tx.pose.position.x = o.x();
      tx.pose.position.y = o.y();
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
  tf2::Quaternion nominal_quat_ = tf2::Quaternion(0, 0, 0, 1);

  packet::RobotInfo<MM_DOF> last_{};
  std::vector<std::string> arm_names_;
  double act_arm_[ARM_DOF], nom_arm_[ARM_DOF];

  XddpLink odom_link_, cmd_link_, status_link_;   // self-healing XDDP endpoints

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_fixed_, sub_rear_, sub_front_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_rear_tw_, sub_front_tw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr act_joints_pub_, nom_joints_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViveXddpNode>();
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->step();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
