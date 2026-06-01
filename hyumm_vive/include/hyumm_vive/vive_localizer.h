// ViveLocalizer -- header-only localization library (the "vive" role).
//
// Turns RAW libsurvive tracker poses/twists (in the backend "libsurvive_world"
// frame) into the hyumm vive_world: it anchors on a fixed reference tracker
// (fixed pose DEFINES vive_world) and computes the mobile-base center pose +
// twist from the rear/front trackers. Pure computation -- no ROS node, no
// topics. The hyumm_nrt node feeds it and broadcasts the TF / sends XDDP.
//
// Math is the verified version from the earlier vive_world_anchor / mobile_base
// nodes (anchor T = T_target * inv(T_ref); base center from offsets with
// occlusion fusion; body-twist transfer to the base center; multi-turn yaw).

#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace hyumm_vive {

inline double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class ViveLocalizer {
public:
  struct Config {
    std::string reference = "tracker_fixed";
    std::string rear = "tracker_mobile_rear";
    std::string front = "tracker_mobile_front";
    std::string world_frame = "vive_world";
    std::string source_frame = "libsurvive_world";
    std::string base_frame = "mobile_base";
    tf::Vector3 anchor_pos = tf::Vector3(1.5, 0.3, 0.9);
    tf::Quaternion anchor_quat = tf::Quaternion(0, 0, 0, 1);  // identity: fixed axes = vive_world axes
    int settle_samples = 30;
    int warmup_samples = 0;   // discard early jittery reference frames
    double orx = -0.395, ory = -0.235, ofx = 0.420, ofy = 0.235;  // tracker offsets in base frame
    double occlusion_timeout = 0.1;  // s
  };

  void configure(const Config& c) {
    cfg_ = c;
    line_angle_ = std::atan2(cfg_.ofy - cfg_.ory, cfg_.ofx - cfg_.orx);
  }
  const Config& config() const { return cfg_; }

  // --- inputs (poses/twists are in the backend source_frame) ---
  void setPose(const std::string& name, const geometry_msgs::Pose& p,
               const ros::Time& t) {
    tf::Transform T;
    tf::poseMsgToTF(p, T);
    pose_[name] = T;
    pose_time_[name] = t;
  }
  void setTwist(const std::string& name, const geometry_msgs::Twist& v) {
    twist_[name] = v;
  }

  // --- anchor ---
  bool locked() const { return locked_; }
  size_t refSamples() const { return ref_samples_.size(); }
  const tf::Transform& anchorTF() const { return T_anchor_; }  // source -> world

  // Collect the reference and lock once enough samples are in. Call each tick
  // while !locked(). Returns true the moment it locks (so the caller can log).
  bool tryLock() {
    if (locked_) return false;
    auto it = pose_.find(cfg_.reference);
    if (it == pose_.end()) return false;
    if (warmup_seen_ < cfg_.warmup_samples) { ++warmup_seen_; return false; }
    ref_samples_.push_back(it->second);
    if (static_cast<int>(ref_samples_.size()) < std::max(1, cfg_.settle_samples))
      return false;

    // mean position, sign-aligned normalized mean quaternion
    tf::Vector3 psum(0, 0, 0);
    tf::Quaternion q0 = ref_samples_.front().getRotation();
    double qx = 0, qy = 0, qz = 0, qw = 0;
    for (const auto& T : ref_samples_) {
      psum += T.getOrigin();
      tf::Quaternion q = T.getRotation();
      if (q.dot(q0) < 0.0) q = tf::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      qx += q.x(); qy += q.y(); qz += q.z(); qw += q.w();
    }
    double n = static_cast<double>(ref_samples_.size());
    tf::Quaternion rq(qx, qy, qz, qw);
    if (rq.length2() < 1e-9) rq = q0;
    rq.normalize();
    tf::Transform T_ref(rq, psum / n);

    tf::Transform target(cfg_.anchor_quat.normalized(), cfg_.anchor_pos);
    T_anchor_ = target * T_ref.inverse();   // source -> world
    locked_ = true;
    return true;
  }

  // Anchored pose of a tracker in vive_world. false if unseen or not locked.
  bool trackerInWorld(const std::string& name, tf::Transform& out) const {
    if (!locked_) return false;
    auto it = pose_.find(name);
    if (it == pose_.end()) return false;
    out = T_anchor_ * it->second;
    return true;
  }

  // Mobile-base center in vive_world. Returns false if unobservable / not locked.
  // Fills T_base (world->base, yaw-planar), base_twist (in vive_world ref frame),
  // and continuous_yaw (multi-turn, for the controller).
  bool mobileBase(const ros::Time& now, tf::Transform& T_base,
                  geometry_msgs::Twist& base_twist, double& continuous_yaw) {
    if (!locked_) return false;
    bool rear = fresh(cfg_.rear, now);
    bool front = fresh(cfg_.front, now);
    if (!rear && !front) return false;

    tf::Transform T_rear, T_front, T_active;
    if (rear) T_rear = T_anchor_ * pose_[cfg_.rear];
    if (front) T_front = T_anchor_ * pose_[cfg_.front];

    double bx, by, bz, yaw;
    double aoff_x, aoff_y;
    const geometry_msgs::Twist* atwist;

    if (rear && front) {
      double rx = T_rear.getOrigin().x(), ry = T_rear.getOrigin().y();
      double fx = T_front.getOrigin().x(), fy = T_front.getOrigin().y();
      yaw = normalizeAngle(std::atan2(fy - ry, fx - rx) - line_angle_);
      double c = std::cos(yaw), s = std::sin(yaw);
      double bxr = rx - (c * cfg_.orx - s * cfg_.ory);
      double byr = ry - (s * cfg_.orx + c * cfg_.ory);
      double bxf = fx - (c * cfg_.ofx - s * cfg_.ofy);
      double byf = fy - (s * cfg_.ofx + c * cfg_.ofy);
      bx = 0.5 * (bxr + bxf); by = 0.5 * (byr + byf);
      bz = 0.5 * (T_rear.getOrigin().z() + T_front.getOrigin().z());
      T_active = T_rear; aoff_x = cfg_.orx; aoff_y = cfg_.ory; atwist = &twist_[cfg_.rear];
    } else if (rear) {
      tf::Transform off(tf::Quaternion(0, 0, 0, 1), tf::Vector3(cfg_.orx, cfg_.ory, 0));
      tf::Transform Tb = T_rear * off.inverse();
      bx = Tb.getOrigin().x(); by = Tb.getOrigin().y(); bz = Tb.getOrigin().z();
      double r, p; Tb.getBasis().getRPY(r, p, yaw);
      T_active = T_rear; aoff_x = cfg_.orx; aoff_y = cfg_.ory; atwist = &twist_[cfg_.rear];
    } else {
      tf::Transform off(tf::Quaternion(0, 0, 0, 1), tf::Vector3(cfg_.ofx, cfg_.ofy, 0));
      tf::Transform Tb = T_front * off.inverse();
      bx = Tb.getOrigin().x(); by = Tb.getOrigin().y(); bz = Tb.getOrigin().z();
      double r, p; Tb.getBasis().getRPY(r, p, yaw);
      T_active = T_front; aoff_x = cfg_.ofx; aoff_y = cfg_.ofy; atwist = &twist_[cfg_.front];
    }

    tf::Quaternion q; q.setRPY(0, 0, yaw);
    T_base = tf::Transform(q, tf::Vector3(bx, by, bz));

    // multi-turn yaw
    if (!yaw_init_) { acc_yaw_ = yaw; prev_yaw_ = yaw; yaw_init_ = true; }
    else { acc_yaw_ += normalizeAngle(yaw - prev_yaw_); prev_yaw_ = yaw; }
    continuous_yaw = acc_yaw_;

    // body twist -> base center velocity, in vive_world ref frame
    tf::Vector3 vlb(atwist->linear.x, atwist->linear.y, atwist->linear.z);
    tf::Vector3 vab(atwist->angular.x, atwist->angular.y, atwist->angular.z);
    tf::Matrix3x3 R = T_active.getBasis();
    tf::Vector3 vtr = R * vlb, wr = R * vab;
    double cy = std::cos(yaw), sy = std::sin(yaw);
    tf::Vector3 roff(cy * aoff_x - sy * aoff_y, sy * aoff_x + cy * aoff_y, 0);
    tf::Vector3 vbr = vtr - wr.cross(roff);
    base_twist.linear.x = vbr.x(); base_twist.linear.y = vbr.y(); base_twist.linear.z = 0.0;
    base_twist.angular.x = 0.0; base_twist.angular.y = 0.0; base_twist.angular.z = wr.z();
    return true;
  }

private:
  bool fresh(const std::string& name, const ros::Time& now) const {
    auto it = pose_time_.find(name);
    if (it == pose_time_.end()) return false;
    return (now - it->second).toSec() < cfg_.occlusion_timeout;
  }

  Config cfg_;
  double line_angle_ = 0.0;
  bool locked_ = false;
  int warmup_seen_ = 0;
  std::vector<tf::Transform> ref_samples_;
  tf::Transform T_anchor_;
  std::map<std::string, tf::Transform> pose_;
  std::map<std::string, ros::Time> pose_time_;
  std::map<std::string, geometry_msgs::Twist> twist_;
  bool yaw_init_ = false;
  double acc_yaw_ = 0.0, prev_yaw_ = 0.0;
};

}  // namespace hyumm_vive
