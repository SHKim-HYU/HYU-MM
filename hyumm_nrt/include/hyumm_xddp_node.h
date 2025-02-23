#ifndef HYUMM_XDDP_NODE_H_
#define HYUMM_XDDP_NODE_H_

#include "ros/ros.h"
#include <thread>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "xddp_ros.h"

int tx_odom_sockfd;
int rx_cmd_vel_sockfd, rx_odom_sockfd, rx_joy_sockfd;

char *devname;
int ret, ret_cmd_vel, ret_twist, ret_odom, ret_joy;
nav_msgs::Odometry tx_odom_msg;
geometry_msgs::PoseStamped rx_pose_msg;
geometry_msgs::Twist rx_cmd_vel_msg;
geometry_msgs::TwistStamped rx_twist_msg;
sensor_msgs::Joy rx_joy_msg;


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