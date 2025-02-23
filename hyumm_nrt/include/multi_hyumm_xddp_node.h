#ifndef HYUMM_XDDP_NODE_H_
#define HYUMM_XDDP_NODE_H_

#include "ros/ros.h"
#include <thread>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "xddp_ros.h"

int odom_sockfd;
int cmd_vel_sockfd;

char *devname;
int ret;
nav_msgs::Odometry odom_msg;
geometry_msgs::Twist twist_msg;

size_t BUFLEN_ODOM = sizeof(packet::Odometry);
size_t BUFLEN_TWIST = sizeof(packet::Twist);

struct packet::Odometry *odom_nrt = (packet::Odometry *)malloc(BUFLEN_ODOM);
struct packet::Twist *twist_nrt = (packet::Twist *)malloc(BUFLEN_TWIST);

#endif  // /* HYUMM_XDDP_NODE_H_ */