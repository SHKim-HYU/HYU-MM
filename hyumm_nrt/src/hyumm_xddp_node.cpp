// hyumm_xddp_node -- ROS2 port.
//
// Reads odom (base pose+twist) the RT controller sends on /dev/rtp<STATE_INFO_1>
// in a background thread and republishes it as nav_msgs/Odometry on /odom. The
// XDDP transport is byte-identical to the Noetic version. (The cmd_vel / joy
// forwarding callbacks are kept commented-out exactly as in the original.)

#include "hyumm_xddp_node.h"

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

	tx_odom_sockfd = open(devname, O_RDONLY);
	free(devname);
	if (tx_odom_sockfd < 0)
		fail("open");

    while (rclcpp::ok())
    {
        /* Get the next message from realtime_thread. */
		ret = read(tx_odom_sockfd, (void *)tx_odom_nrt, BUFLEN_ODOM);

        if(ret>0)
        {
            tx_odom_msg.header.stamp = g_node->now();
            tx_odom_msg.header.frame_id = "odom";
            tx_odom_msg.pose.pose.position.x = tx_odom_nrt->pose.position.x;
            tx_odom_msg.pose.pose.position.y = tx_odom_nrt->pose.position.y;
            tx_odom_msg.pose.pose.position.z = tx_odom_nrt->pose.position.z;
            tx_odom_msg.pose.pose.orientation.x = tx_odom_nrt->pose.orientation.x;
            tx_odom_msg.pose.pose.orientation.y = tx_odom_nrt->pose.orientation.y;
            tx_odom_msg.pose.pose.orientation.z = tx_odom_nrt->pose.orientation.z;
            tx_odom_msg.pose.pose.orientation.w = tx_odom_nrt->pose.orientation.w;
            tx_odom_msg.twist.twist.linear.x = tx_odom_nrt->twist.linear.x;
            tx_odom_msg.twist.twist.linear.y = tx_odom_nrt->twist.linear.y;
            tx_odom_msg.twist.twist.linear.z = tx_odom_nrt->twist.linear.z;
            tx_odom_msg.twist.twist.angular.x = tx_odom_nrt->twist.angular.x;
            tx_odom_msg.twist.twist.angular.y = tx_odom_nrt->twist.angular.y;
            tx_odom_msg.twist.twist.angular.z = tx_odom_nrt->twist.angular.z;

            pubOdometry->publish(tx_odom_msg);
        }
    }
    close(tx_odom_sockfd);
}

void cmdvelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    rx_cmd_vel_nrt->linear.x = msg->linear.x;
    rx_cmd_vel_nrt->linear.y = msg->linear.y;
    rx_cmd_vel_nrt->linear.z = msg->linear.z;
    rx_cmd_vel_nrt->angular.x = msg->angular.x;
    rx_cmd_vel_nrt->angular.y = msg->angular.y;
    rx_cmd_vel_nrt->angular.z = msg->angular.z;
}

void twistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
    rx_odom_nrt->twist.linear.x = msg->twist.linear.x;
    rx_odom_nrt->twist.linear.y = msg->twist.linear.y;
    rx_odom_nrt->twist.linear.z = msg->twist.linear.z;
    rx_odom_nrt->twist.angular.x = msg->twist.angular.x;
    rx_odom_nrt->twist.angular.y = msg->twist.angular.y;
    rx_odom_nrt->twist.angular.z = msg->twist.angular.z;
}

void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    rx_odom_nrt->pose.position.x = msg->pose.position.x;
    rx_odom_nrt->pose.position.y = msg->pose.position.y;
    rx_odom_nrt->pose.position.z = msg->pose.position.z;
    rx_odom_nrt->pose.orientation.x = msg->pose.orientation.x;
    rx_odom_nrt->pose.orientation.y = msg->pose.orientation.y;
    rx_odom_nrt->pose.orientation.z = msg->pose.orientation.z;
    rx_odom_nrt->pose.orientation.w = msg->pose.orientation.w;
}

void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
    for(int i = 0; i<BUTTON_NUM; i++)
        rx_joy_nrt->buttons[i] = msg->buttons[i];

    ret_joy = write(rx_joy_sockfd, (void *)rx_joy_nrt, BUFLEN_JOY);
    ret_odom = write(rx_odom_sockfd, (void *)rx_odom_nrt, BUFLEN_ODOM);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("xddp_nrt");

    auto pubOdometry = g_node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    // auto subCmdVel = g_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1,cmdvelCallback);
    // auto subTwist = g_node->create_subscription<geometry_msgs::msg::TwistStamped>("/vive/right_controller/twist",1,twistCallback);
    // auto subPose = g_node->create_subscription<geometry_msgs::msg::PoseStamped>("/vive/right_controller/pose",1,poseCallback);
    // auto subJoy = g_node->create_subscription<sensor_msgs::msg::Joy>("/vive/right_controller/joy",1,joyCallback);

    std::thread odom_thread(readOdomData, pubOdometry);

    rclcpp::spin(g_node);

    odom_thread.join();

    rclcpp::shutdown();
    return 0;
}
