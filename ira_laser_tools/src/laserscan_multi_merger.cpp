// laserscan_multi_merger -- ROS2 port (dependency-free).
//
// Merges several sensor_msgs/LaserScan topics into one PointCloud2 + one
// re-binned LaserScan, all expressed in a common destination frame.
//
// The Noetic node leaned on laser_geometry + PCL + pcl_ros + dynamic_reconfigure.
// Those are not installed in this ROS2 Galactic environment, so the projection
// (polar -> cartesian), the TF transform to the destination frame, the point
// merge and the re-bin are reimplemented directly on top of tf2 + the
// sensor_msgs PointCloud2 iterators. dynamic_reconfigure is replaced by plain
// ROS2 parameters (live-updatable via the on-set-parameters callback). The
// configured laserscan_topics are subscribed directly by name (ROS2 allows
// subscribing before the publisher exists, so no master polling is needed).

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LaserscanMerger : public rclcpp::Node {
 public:
  LaserscanMerger() : rclcpp::Node("laserscan_multi_merger") {
    destination_frame_ = declare_parameter<std::string>("destination_frame", "cart_frame");
    cloud_destination_topic_ = declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    scan_destination_topic_ = declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
    laserscan_topics_ = declare_parameter<std::string>("laserscan_topics", "");
    angle_min_ = declare_parameter<double>("angle_min", -2.36);
    angle_max_ = declare_parameter<double>("angle_max", 2.36);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.0058);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.0333333);
    range_min_ = declare_parameter<double>("range_min", 0.45);
    range_max_ = declare_parameter<double>("range_max", 25.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic_, 1);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic_, 1);

    // Parse the space-separated topic list and subscribe to each by name.
    std::istringstream iss(laserscan_topics_);
    std::string topic;
    while (iss >> topic) input_topics_.push_back(topic);

    if (input_topics_.empty()) {
      RCLCPP_WARN(get_logger(), "laserscan_topics is empty: nothing to merge");
    }
    points_.resize(input_topics_.size());
    modified_.assign(input_topics_.size(), false);
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "subscribing to %s", input_topics_[i].c_str());
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
          input_topics_[i], rclcpp::SensorDataQoS(),
          [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr s) { scanCallback(s, i); }));
    }

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "angle_min") angle_min_ = p.as_double();
            else if (n == "angle_max") angle_max_ = p.as_double();
            else if (n == "angle_increment") angle_increment_ = p.as_double();
            else if (n == "time_increment") time_increment_ = p.as_double();
            else if (n == "scan_time") scan_time_ = p.as_double();
            else if (n == "range_min") range_min_ = p.as_double();
            else if (n == "range_max") range_max_ = p.as_double();
          }
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          return r;
        });

    RCLCPP_INFO(get_logger(),
        "laserscan_multi_merger: %zu input(s) -> %s (PointCloud2) + %s (LaserScan) in frame '%s'",
        input_topics_.size(), cloud_destination_topic_.c_str(),
        scan_destination_topic_.c_str(), destination_frame_.c_str());
  }

 private:
  struct Pt { float x, y, z; };

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, size_t idx) {
    // TF: destination_frame <- scan frame. Sensor->base mounts are rigid
    // (static), so use the latest available transform (robust to scan-stamp vs
    // TF-time skew and to scans published without a stamp).
    tf2::Transform T;
    try {
      const auto ts = tf_buffer_->lookupTransform(
          destination_frame_, scan->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(1.0));
      T = tf2::Transform(
          tf2::Quaternion(ts.transform.rotation.x, ts.transform.rotation.y,
                          ts.transform.rotation.z, ts.transform.rotation.w),
          tf2::Vector3(ts.transform.translation.x, ts.transform.translation.y,
                       ts.transform.translation.z));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s <- %s failed: %s", destination_frame_.c_str(),
                           scan->header.frame_id.c_str(), e.what());
      return;  // TF not ready; drop this scan
    }

    // Project polar -> cartesian (laser frame) -> destination frame.
    std::vector<Pt>& pts = points_[idx];
    pts.clear();
    pts.reserve(scan->ranges.size());
    for (size_t j = 0; j < scan->ranges.size(); ++j) {
      const float r = scan->ranges[j];
      if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;
      const double a = scan->angle_min + j * scan->angle_increment;
      const tf2::Vector3 p_dest = T * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
      pts.push_back({static_cast<float>(p_dest.x()),
                     static_cast<float>(p_dest.y()),
                     static_cast<float>(p_dest.z())});
    }
    modified_[idx] = true;

    // Publish once every subscribed scan has contributed at least once.
    for (bool m : modified_) if (!m) return;

    std::vector<Pt> merged;
    size_t total = 0;
    for (const auto& v : points_) total += v.size();
    merged.reserve(total);
    for (const auto& v : points_) merged.insert(merged.end(), v.begin(), v.end());
    for (size_t i = 0; i < modified_.size(); ++i) modified_[i] = false;

    publishCloud(merged, scan->header.stamp);
    publishScan(merged, scan->header.stamp);
  }

  void publishCloud(const std::vector<Pt>& pts, const rclcpp::Time& stamp) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = destination_frame_;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    for (const auto& p : pts) {
      *ix = p.x; *iy = p.y; *iz = p.z;
      ++ix; ++iy; ++iz;
    }
    cloud_pub_->publish(cloud);
  }

  void publishScan(const std::vector<Pt>& pts, const rclcpp::Time& stamp) {
    sensor_msgs::msg::LaserScan out;
    out.header.stamp = stamp;
    out.header.frame_id = destination_frame_;
    out.angle_min = angle_min_;
    out.angle_max = angle_max_;
    out.angle_increment = angle_increment_;
    out.time_increment = time_increment_;
    out.scan_time = scan_time_;
    out.range_min = range_min_;
    out.range_max = range_max_;

    const uint32_t n = std::ceil((out.angle_max - out.angle_min) / out.angle_increment);
    out.ranges.assign(n, out.range_max + 1.0f);
    const double range_min_sq = out.range_min * out.range_min;

    for (const auto& p : pts) {
      if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
      const double range_sq = p.x * p.x + p.y * p.y;
      if (range_sq < range_min_sq) continue;
      const double angle = std::atan2(p.y, p.x);
      if (angle < out.angle_min || angle > out.angle_max) continue;
      const int index = (angle - out.angle_min) / out.angle_increment;
      if (index < 0 || index >= static_cast<int>(n)) continue;
      const float rng = std::sqrt(range_sq);
      if (rng < out.ranges[index]) out.ranges[index] = rng;
    }
    scan_pub_->publish(out);
  }

  std::string destination_frame_, cloud_destination_topic_, scan_destination_topic_, laserscan_topics_;
  double angle_min_, angle_max_, angle_increment_, time_increment_, scan_time_, range_min_, range_max_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
  std::vector<std::string> input_topics_;
  std::vector<std::vector<Pt>> points_;
  std::vector<bool> modified_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanMerger>());
  rclcpp::shutdown();
  return 0;
}
