// vive_world_anchor_node
//
// Application-layer definition of the vive_world frame for HYU-MM.
//
// The vive_libsurvive_ros BACKEND publishes each device's pose in its own
// frame ("libsurvive_world") on /vive/<name>/pose (+ body-frame twist on
// /vive/<name>/twist, frame_id = device name). This node anchors that frame
// onto a known reference: once the fixed reference tracker has been seen for
// `settle_samples` frames, it computes the rigid transform T_anchor such that
// the reference reads exactly (anchor_position, anchor_orientation) -- i.e. the
// reference's OWN pose defines vive_world (anchor_orientation = identity => the
// fixed tracker's mounting axes become the vive_world axes). It then:
//   * broadcasts TF  vive_world -> libsurvive_world  (= T_anchor), so the whole
//     backend TF tree resolves in vive_world, and
//   * republishes each device pose in vive_world on /vive_world/<name>/pose.
//
// TWIST: /vive_world/<name>/twist is the backend body-frame twist relayed
// UNCHANGED (frame_id stays the device body frame BY DESIGN, matching the
// existing consumer). It is NOT "the velocity of the tracker point expressed in
// vive_world": the backend body twist is computed at the libsurvive_world
// position, so only its angular part is strictly invariant to this (translation)
// re-anchor. Do not integrate it as a vive_world point velocity; if a true
// vive_world twist is ever needed, recompute it from the anchored pose.
//
// NOTE: the backend does NOT define vive_world; that is solely this node's job.
// Do not run this alongside the SteamVR vive_tracking_ros stack (it also uses a
// frame literally named 'vive_world').

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class ViveWorldAnchor {
public:
  ViveWorldAnchor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : locked_(false), warmup_seen_(0), drift_count_(0) {
    pnh.param<std::string>("reference", reference_, "tracker_fixed");
    pnh.param<std::string>("world_frame", world_frame_, "vive_world");
    pnh.param<std::string>("source_frame", source_frame_, "libsurvive_world");
    pnh.param<std::string>("input_ns", input_ns_, "/vive");
    pnh.param<std::string>("output_ns", output_ns_, "/vive_world");
    pnh.param("settle_samples", settle_samples_, 30);
    pnh.param("warmup_samples", warmup_samples_, 0);
    pnh.param("publish_twist", publish_twist_, true);
    pnh.param("drift_warn_pos", drift_warn_pos_, 0.03);   // m
    pnh.param("drift_warn_deg", drift_warn_deg_, 3.0);    // deg
    if (settle_samples_ < 1) {
      ROS_WARN("settle_samples < 1; using 1");
      settle_samples_ = 1;
    }
    if (warmup_samples_ < 0) warmup_samples_ = 0;

    std::vector<double> pos, ori;
    if (!pnh.getParam("anchor_position", pos)) {
      ROS_WARN("anchor_position not set; using built-in default (1.5,0.3,0.9) -- "
               "check that the config file is loaded into this node's namespace");
      pos = {1.5, 0.3, 0.9};
    }
    if (pos.size() != 3) {
      ROS_WARN("anchor_position must have 3 elements; using (0,0,0)");
      pos = {0.0, 0.0, 0.0};
    }
    target_.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));

    if (!pnh.getParam("anchor_orientation", ori)) {
      ROS_WARN("anchor_orientation not set; using identity");
      ori = {0.0, 0.0, 0.0, 1.0};
    }
    if (ori.size() == 4) {
      tf::Quaternion q(ori[0], ori[1], ori[2], ori[3]);
      if (q.length2() < 1e-9) {
        ROS_WARN("anchor_orientation is near-zero; using identity");
        q = tf::Quaternion(0, 0, 0, 1);
      }
      target_.setRotation(q.normalized());
    } else if (ori.size() == 3) {
      tf::Quaternion q;
      q.setRPY(ori[0], ori[1], ori[2]);
      target_.setRotation(q);
    } else {
      ROS_WARN("anchor_orientation must be 4 (xyzw) or 3 (rpy); using identity");
      target_.setRotation(tf::Quaternion(0, 0, 0, 1));
    }

    std::vector<std::string> trackers;
    if (!pnh.getParam("trackers", trackers)) {
      ROS_WARN("trackers not set; using default [tracker_fixed, "
               "tracker_mobile_rear, tracker_mobile_front]");
      trackers = {"tracker_fixed", "tracker_mobile_rear", "tracker_mobile_front"};
    }

    for (const std::string& name : trackers) {
      pose_pubs_[name] = nh.advertise<geometry_msgs::PoseStamped>(
          output_ns_ + "/" + name + "/pose", 10);
      pose_subs_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
          input_ns_ + "/" + name + "/pose", 10,
          boost::bind(&ViveWorldAnchor::poseCb, this, _1, name)));
      if (publish_twist_) {
        twist_pubs_[name] = nh.advertise<geometry_msgs::TwistStamped>(
            output_ns_ + "/" + name + "/twist", 10);
        twist_subs_.push_back(nh.subscribe<geometry_msgs::TwistStamped>(
            input_ns_ + "/" + name + "/twist", 10,
            boost::bind(&ViveWorldAnchor::twistCb, this, _1, name)));
      }
    }

    tf::Vector3 t = target_.getOrigin();
    ROS_INFO("vive_world_anchor: ref='%s' target=(%.3f,%.3f,%.3f) settle=%d "
             "warmup=%d -> defining %s from %s. Waiting for reference ...",
             reference_.c_str(), t.x(), t.y(), t.z(), settle_samples_,
             warmup_samples_, world_frame_.c_str(), source_frame_.c_str());
  }

  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg,
              const std::string& name) {
    tf::Transform T_dev;
    tf::poseMsgToTF(msg->pose, T_dev);

    if (!locked_) {
      if (name == reference_) {
        if (warmup_seen_ < warmup_samples_) {       // skip early jittery frames
          ++warmup_seen_;
          return;
        }
        ref_samples_.push_back(T_dev);
        if (static_cast<int>(ref_samples_.size()) >= settle_samples_) lockAnchor();
      }
      return;  // publish nothing until the anchor is locked
    }

    if (name == reference_) checkDrift(T_dev);

    tf::Transform T_anchored = T_anchor_ * T_dev;
    geometry_msgs::PoseStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = world_frame_;
    tf::poseTFToMsg(T_anchored, out.pose);
    pose_pubs_[name].publish(out);
  }

  void twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg,
               const std::string& name) {
    if (!locked_) return;
    // Relay the backend body-frame twist unchanged (see file header).
    twist_pubs_[name].publish(*msg);
  }

  void lockAnchor() {
    // Average the reference samples: mean position, sign-aligned normalized
    // mean quaternion (valid for a near-stationary reference).
    tf::Vector3 pos_sum(0, 0, 0);
    tf::Quaternion q0 = ref_samples_.front().getRotation();
    double qx = 0, qy = 0, qz = 0, qw = 0;
    for (const tf::Transform& T : ref_samples_) {
      pos_sum += T.getOrigin();
      tf::Quaternion q = T.getRotation();
      if (q.dot(q0) < 0.0) q = tf::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      qx += q.x(); qy += q.y(); qz += q.z(); qw += q.w();
    }
    const double n = static_cast<double>(ref_samples_.size());
    tf::Vector3 ref_pos = pos_sum / n;
    tf::Quaternion ref_q(qx, qy, qz, qw);
    if (ref_q.length2() < 1e-9) ref_q = q0;          // degenerate guard
    ref_q.normalize();
    tf::Transform T_ref(ref_q, ref_pos);

    // T_anchor maps libsurvive_world -> vive_world so that T_anchor*T_ref = target.
    T_anchor_ = target_ * T_ref.inverse();
    locked_ = true;

    // Lock residual (should be ~0): re-anchor the averaged reference and compare.
    double dp, dd;
    residual(T_ref, dp, dd);
    tf::Vector3 t = target_.getOrigin();
    ROS_INFO("vive_world anchor LOCKED on '%s' over %.0f samples: ref pos "
             "(%.3f %.3f %.3f) -> target (%.3f %.3f %.3f). Lock residual: "
             "%.4f m, %.3f deg. Publishing %s.",
             reference_.c_str(), n, ref_pos.x(), ref_pos.y(), ref_pos.z(),
             t.x(), t.y(), t.z(), dp, dd, output_ns_.c_str());
  }

  // Compare where the reference currently lands (T_anchor_*T_dev) against target.
  void residual(const tf::Transform& T_dev, double& pos_err, double& deg_err) const {
    tf::Transform r = T_anchor_ * T_dev;
    pos_err = (r.getOrigin() - target_.getOrigin()).length();
    double a = r.getRotation().angleShortestPath(target_.getRotation());
    deg_err = std::fabs(a) * 180.0 / M_PI;
  }

  // After lock the fixed reference must stay put; if it drifts the world is stale.
  void checkDrift(const tf::Transform& T_dev) {
    double dp, dd;
    residual(T_dev, dp, dd);
    if (dp > drift_warn_pos_ || dd > drift_warn_deg_) {
      if (++drift_count_ >= 30) {
        ROS_WARN_THROTTLE(5.0,
            "vive_world anchor STALE: reference '%s' moved %.3f m / %.2f deg from "
            "its locked pose -- the fixed tracker was bumped? Restart to re-anchor.",
            reference_.c_str(), dp, dd);
      }
    } else if (drift_count_ > 0) {
      drift_count_ = 0;
    }
  }

  void spin() {
    if (locked_) {
      // parent = vive_world, child = libsurvive_world, transform = T_anchor.
      br_.sendTransform(tf::StampedTransform(T_anchor_, ros::Time::now(),
                                             world_frame_, source_frame_));
    } else {
      ROS_WARN_THROTTLE(2.0,
          "vive_world anchor NOT locked: %zu/%d reference samples from '%s' -- "
          "is the fixed tracker tracking?",
          ref_samples_.size(), settle_samples_, reference_.c_str());
    }
  }

private:
  std::string reference_, world_frame_, source_frame_, input_ns_, output_ns_;
  int settle_samples_, warmup_samples_, warmup_seen_, drift_count_;
  double drift_warn_pos_, drift_warn_deg_;
  bool publish_twist_;
  bool locked_;
  tf::Transform target_, T_anchor_;
  std::vector<tf::Transform> ref_samples_;
  std::map<std::string, ros::Publisher> pose_pubs_, twist_pubs_;
  std::vector<ros::Subscriber> pose_subs_, twist_subs_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vive_world_anchor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ViveWorldAnchor anchor(nh, pnh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    anchor.spin();
    rate.sleep();
  }
  return 0;
}
