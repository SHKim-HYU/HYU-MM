#ifndef HYUMM_XDDP_NODE_H_
#define HYUMM_XDDP_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "xddp_ros.h"

int tx_odom_sockfd;
int rx_cmd_vel_sockfd, rx_odom_sockfd, rx_joy_sockfd;

char *devname;
int ret, ret_cmd_vel, ret_twist, ret_odom, ret_joy;
nav_msgs::msg::Odometry tx_odom_msg;
geometry_msgs::msg::PoseStamped rx_pose_msg;
geometry_msgs::msg::Twist rx_cmd_vel_msg;
geometry_msgs::msg::TwistStamped rx_twist_msg;
sensor_msgs::msg::Joy rx_joy_msg;


size_t BUFLEN_ODOM = sizeof(packet::Odometry);
size_t BUFLEN_TWIST = sizeof(packet::Twist);
size_t BUFLEN_JOY = sizeof(packet::Joy);

// Command to MM
struct packet::Twist *rx_cmd_vel_nrt = (packet::Twist *)malloc(BUFLEN_TWIST);
struct packet::Odometry *rx_odom_nrt = (packet::Odometry *)malloc(BUFLEN_ODOM);
struct packet::Joy *rx_joy_nrt = (packet::Joy *)malloc(BUFLEN_JOY);

// Status from MM
struct packet::Odometry *tx_odom_nrt = (packet::Odometry *)malloc(BUFLEN_ODOM);


#endif  // /* HYUMM_XDDP_NODE_H_ */
