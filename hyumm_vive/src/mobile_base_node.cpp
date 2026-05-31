// mobile_base_node
//
// Computes the mobile-base center pose in vive_world from the rear/front mobile
// trackers and publishes it as TF (vive_world -> mobile_base) + a PoseStamped,
// so the base shows up in RViz alongside the trackers / lighthouses.
//
// Input is the ALREADY-ANCHORED vive_world poses from hyumm_vive's
// vive_world_anchor_node (/vive_world/<name>/pose), so no calibration is needed
// here. The base-center geometry (tracker offsets, occlusion fusion, heading)
// mirrors hyumm_nrt/hyumm_scan_xddp_node so the visualised base matches what the
// XDDP path sends. (scan_xddp still computes its own copy for the RT controller;
// keep the offsets here in sync with it.)

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <cmath>
#include <string>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

static double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class MobileBaseNode {
public:
  MobileBaseNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : rear_ok_(false), front_ok_(false) {
    pnh.param<std::string>("world_frame", world_frame_, "vive_world");
    pnh.param<std::string>("base_frame", base_frame_, "mobile_base");
    pnh.param<std::string>("input_ns", input_ns_, "/vive_world");
    pnh.param<std::string>("rear", rear_name_, "tracker_mobile_rear");
    pnh.param<std::string>("front", front_name_, "tracker_mobile_front");
    // Tracker positions relative to the base center, in the base frame (must
    // match hyumm_scan_xddp_node's OFFSET_* constants).
    pnh.param("offset_rear_x", orx_, -0.395);
    pnh.param("offset_rear_y", ory_, -0.235);
    pnh.param("offset_front_x", ofx_, 0.420);
    pnh.param("offset_front_y", ofy_, 0.235);
    pnh.param("occlusion_timeout", occ_timeout_, 0.1);
    pnh.param("publish_pose", publish_pose_, true);
    line_angle_ = std::atan2(ofy_ - ory_, ofx_ - orx_);

    rear_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        input_ns_ + "/" + rear_name_ + "/pose", 10,
        boost::bind(&MobileBaseNode::poseCb, this, _1, true));
    front_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        input_ns_ + "/" + front_name_ + "/pose", 10,
        boost::bind(&MobileBaseNode::poseCb, this, _1, false));
    if (publish_pose_)
      pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
          input_ns_ + "/" + base_frame_ + "/pose", 10);

    ROS_INFO("mobile_base_node: %s from %s/%s (offsets rear(%.3f,%.3f) "
             "front(%.3f,%.3f)) in %s",
             base_frame_.c_str(), rear_name_.c_str(), front_name_.c_str(),
             orx_, ory_, ofx_, ofy_, world_frame_.c_str());
  }

  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg, bool is_rear) {
    if (is_rear) {
      tf::poseMsgToTF(msg->pose, T_rear_);
      rear_time_ = ros::Time::now();
      rear_ok_ = true;
    } else {
      tf::poseMsgToTF(msg->pose, T_front_);
      front_time_ = ros::Time::now();
      front_ok_ = true;
    }
  }

  void spin() {
    ros::Time now = ros::Time::now();
    bool rear = rear_ok_ && (now - rear_time_).toSec() < occ_timeout_;
    bool front = front_ok_ && (now - front_time_).toSec() < occ_timeout_;
    if (!rear && !front) return;  // base unobservable -> publish nothing

    double bx, by, bz, yaw;

    if (rear && front) {
      // Both visible: heading from the rear->front vector, base center is the
      // average of the two single-tracker estimates (yaw-planar offsets).
      double rx = T_rear_.getOrigin().x(), ry = T_rear_.getOrigin().y();
      double fx = T_front_.getOrigin().x(), fy = T_front_.getOrigin().y();
      yaw = normalizeAngle(std::atan2(fy - ry, fx - rx) - line_angle_);
      double c = std::cos(yaw), s = std::sin(yaw);
      double bxr = rx - (c * orx_ - s * ory_);
      double byr = ry - (s * orx_ + c * ory_);
      double bxf = fx - (c * ofx_ - s * ofy_);
      double byf = fy - (s * ofx_ + c * ofy_);
      bx = 0.5 * (bxr + bxf);
      by = 0.5 * (byr + byf);
      bz = 0.5 * (T_rear_.getOrigin().z() + T_front_.getOrigin().z());
    } else if (rear) {
      // Rear only: base = rear * inv(offset), yaw from the rear tracker pose.
      tf::Transform T_off(tf::Quaternion(0, 0, 0, 1), tf::Vector3(orx_, ory_, 0));
      tf::Transform T_base = T_rear_ * T_off.inverse();
      bx = T_base.getOrigin().x();
      by = T_base.getOrigin().y();
      bz = T_base.getOrigin().z();
      double roll, pitch;
      T_base.getBasis().getRPY(roll, pitch, yaw);
    } else {
      // Front only.
      tf::Transform T_off(tf::Quaternion(0, 0, 0, 1), tf::Vector3(ofx_, ofy_, 0));
      tf::Transform T_base = T_front_ * T_off.inverse();
      bx = T_base.getOrigin().x();
      by = T_base.getOrigin().y();
      bz = T_base.getOrigin().z();
      double roll, pitch;
      T_base.getBasis().getRPY(roll, pitch, yaw);
    }

    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf::Transform T_base(q, tf::Vector3(bx, by, bz));
    br_.sendTransform(
        tf::StampedTransform(T_base, now, world_frame_, base_frame_));

    if (publish_pose_) {
      geometry_msgs::PoseStamped out;
      out.header.stamp = now;
      out.header.frame_id = world_frame_;
      tf::poseTFToMsg(T_base, out.pose);
      pose_pub_.publish(out);
    }
  }

private:
  std::string world_frame_, base_frame_, input_ns_, rear_name_, front_name_;
  double orx_, ory_, ofx_, ofy_, line_angle_, occ_timeout_;
  bool publish_pose_, rear_ok_, front_ok_;
  tf::Transform T_rear_, T_front_;
  ros::Time rear_time_, front_time_;
  ros::Subscriber rear_sub_, front_sub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mobile_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  MobileBaseNode node(nh, pnh);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    node.spin();
    rate.sleep();
  }
  return 0;
}
