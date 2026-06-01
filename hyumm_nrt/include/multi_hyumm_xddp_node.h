#ifndef HYUMM_XDDP_NODE_H_
#define HYUMM_XDDP_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "xddp_ros.h"

int odom_sockfd;
int cmd_vel_sockfd;

char *devname;
int ret;
nav_msgs::msg::Odometry odom_msg;
geometry_msgs::msg::Twist twist_msg;

size_t BUFLEN_ODOM = sizeof(packet::Odometry);
size_t BUFLEN_TWIST = sizeof(packet::Twist);

struct packet::Odometry *odom_nrt = (packet::Odometry *)malloc(BUFLEN_ODOM);
struct packet::Twist *twist_nrt = (packet::Twist *)malloc(BUFLEN_TWIST);

#endif  // /* HYUMM_XDDP_NODE_H_ */
