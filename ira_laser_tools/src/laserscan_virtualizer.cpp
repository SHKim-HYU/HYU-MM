// laserscan_virtualizer -- ROS2 port (dependency-free).
//
// Takes one PointCloud2 and re-projects it into a virtual sensor_msgs/LaserScan
// for each requested output frame (a "virtual laser" sitting at each frame).
//
// Like the merger, the Noetic node used laser_geometry + PCL + pcl_ros +
// dynamic_reconfigure; here the per-frame TF transform and the z~0 re-bin are
// done directly with tf2 + the sensor_msgs PointCloud2 iterators, and
// dynamic_reconfigure is replaced by plain ROS2 parameters.

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

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LaserscanVirtualizer : public rclcpp::Node {
 public:
  LaserscanVirtualizer() : rclcpp::Node("laserscan_virtualizer") {
    base_frame_ = declare_parameter<std::string>("base_frame", "cart_frame");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_pcd");
    output_laser_topic_ = declare_parameter<std::string>("output_laser_topic", "");
    virtual_laser_scan_ = declare_parameter<std::string>("virtual_laser_scan", "");
    angle_min_ = declare_parameter<double>("angle_min", -2.36);
    angle_max_ = declare_parameter<double>("angle_max", 2.36);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.0058);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.0333333);
    range_min_ = declare_parameter<double>("range_min", 0.45);
    range_max_ = declare_parameter<double>("range_max", 25.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::istringstream iss(virtual_laser_scan_);
    std::string frame;
    while (iss >> frame) output_frames_.push_back(frame);

    for (size_t i = 0; i < output_frames_.size(); ++i) {
      const std::string topic = output_laser_topic_.empty() ? output_frames_[i] : output_laser_topic_;
      pubs_.push_back(create_publisher<sensor_msgs::msg::LaserScan>(topic, 1));
      RCLCPP_INFO(get_logger(), "virtual scan for frame '%s' -> topic '%s'",
                  output_frames_[i].c_str(), topic.c_str());
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LaserscanVirtualizer::cloudCallback, this, std::placeholders::_1));

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
  }

 private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    for (size_t i = 0; i < output_frames_.size(); ++i) {
      tf2::Transform T;
      try {
        const auto ts = tf_buffer_->lookupTransform(
            output_frames_[i], cloud->header.frame_id,
            tf2::TimePointZero, tf2::durationFromSec(1.0));
        T = tf2::Transform(
            tf2::Quaternion(ts.transform.rotation.x, ts.transform.rotation.y,
                            ts.transform.rotation.z, ts.transform.rotation.w),
            tf2::Vector3(ts.transform.translation.x, ts.transform.translation.y,
                         ts.transform.translation.z));
      } catch (const tf2::TransformException&) {
        continue;
      }

      sensor_msgs::msg::LaserScan out;
      out.header.stamp = cloud->header.stamp;
      out.header.frame_id = output_frames_[i];
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

      sensor_msgs::PointCloud2ConstIterator<float> ix(*cloud, "x"), iy(*cloud, "y"), iz(*cloud, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (std::isnan(*ix) || std::isnan(*iy) || std::isnan(*iz)) continue;
        const tf2::Vector3 p = T * tf2::Vector3(*ix, *iy, *iz);
        const double x = p.x(), y = p.y();
        const double range_sq = x * x + y * y;
        if (range_sq < range_min_sq) continue;
        const double angle = std::atan2(y, x);
        if (angle < out.angle_min || angle > out.angle_max) continue;
        const int index = (angle - out.angle_min) / out.angle_increment;
        if (index < 0 || index >= static_cast<int>(n)) continue;
        const float rng = std::sqrt(range_sq);
        if (rng < out.ranges[index]) out.ranges[index] = rng;
      }
      pubs_[i]->publish(out);
    }
  }

  std::string base_frame_, cloud_topic_, output_laser_topic_, virtual_laser_scan_;
  double angle_min_, angle_max_, angle_increment_, time_increment_, scan_time_, range_min_, range_max_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> pubs_;
  std::vector<std::string> output_frames_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanVirtualizer>());
  rclcpp::shutdown();
  return 0;
}
