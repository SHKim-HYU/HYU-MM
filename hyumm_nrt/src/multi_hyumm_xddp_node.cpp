// multi_hyumm_xddp_node -- ROS2 port.
//
// Reads odom from RT on /dev/rtp<STATE_INFO_1> in a thread -> /odom, and
// forwards /cmd_vel onto /dev/rtp<CMD_INFO_1>. XDDP transport byte-identical to
// the Noetic version.

#include "multi_hyumm_xddp_node.h"

static rclcpp::Node::SharedPtr g_node;

static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void readOdomData(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry)
{
	if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_STATE_INFO_1) < 0)
		fail("asprintf");

	odom_sockfd = open(devname, O_RDONLY);
	free(devname);
	if (odom_sockfd < 0)
		fail("open");

    while (rclcpp::ok())
    {
        /* Get the next message from realtime_thread. */
		ret = read(odom_sockfd, (void *)odom_nrt, BUFLEN_ODOM);

        if(ret>0)
        {
            odom_msg.header.stamp = g_node->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.pose.pose.position.x = odom_nrt->pose.position.x;
            odom_msg.pose.pose.position.y = odom_nrt->pose.position.y;
            odom_msg.pose.pose.position.z = odom_nrt->pose.position.z;
            odom_msg.pose.pose.orientation.x = odom_nrt->pose.orientation.x;
            odom_msg.pose.pose.orientation.y = odom_nrt->pose.orientation.y;
            odom_msg.pose.pose.orientation.z = odom_nrt->pose.orientation.z;
            odom_msg.pose.pose.orientation.w = odom_nrt->pose.orientation.w;
            odom_msg.twist.twist.linear.x = odom_nrt->twist.linear.x;
            odom_msg.twist.twist.linear.y = odom_nrt->twist.linear.y;
            odom_msg.twist.twist.linear.z = odom_nrt->twist.linear.z;
            odom_msg.twist.twist.angular.x = odom_nrt->twist.angular.x;
            odom_msg.twist.twist.angular.y = odom_nrt->twist.angular.y;
            odom_msg.twist.twist.angular.z = odom_nrt->twist.angular.z;

            pubOdometry->publish(odom_msg);
        }
    }
    close(odom_sockfd);
}

void twistCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    twist_nrt->linear.x = msg->linear.x;
    twist_nrt->linear.y = msg->linear.y;
    twist_nrt->linear.z = msg->linear.z;
    twist_nrt->angular.x = msg->angular.x;
    twist_nrt->angular.y = msg->angular.y;
    twist_nrt->angular.z = msg->angular.z;

    ret = write(cmd_vel_sockfd, (void *)twist_nrt, BUFLEN_TWIST);
}

int main(int argc, char** argv)
{
    if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_INFO_1) < 0)
		fail("asprintf");

	cmd_vel_sockfd = open(devname, O_RDWR);
	free(devname);
	if (cmd_vel_sockfd < 0)
		fail("open");

    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("xddp_nrt");

    auto pubOdometry = g_node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto subTwist = g_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, twistCallback);

    std::thread odom_thread(readOdomData, pubOdometry);

    rclcpp::spin(g_node);

    odom_thread.join();

    rclcpp::shutdown();
    return 0;
}
