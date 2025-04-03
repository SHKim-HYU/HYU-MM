#include "hyumm_xddp_node.h"

static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}


void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    rx_cmd_vel_nrt->linear.x = msg->linear.x;
    rx_cmd_vel_nrt->linear.y = msg->linear.y;
    rx_cmd_vel_nrt->linear.z = msg->linear.z;
    rx_cmd_vel_nrt->angular.x = msg->angular.x;
    rx_cmd_vel_nrt->angular.y = msg->angular.y;
    rx_cmd_vel_nrt->angular.z = msg->angular.z;
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    rx_odom_nrt->twist.linear.x = msg->twist.linear.x;
    rx_odom_nrt->twist.linear.y = msg->twist.linear.y;
    rx_odom_nrt->twist.linear.z = msg->twist.linear.z;
    rx_odom_nrt->twist.angular.x = msg->twist.angular.x;
    rx_odom_nrt->twist.angular.y = msg->twist.angular.y;
    rx_odom_nrt->twist.angular.z = msg->twist.angular.z;
    // ROS_INFO("twist: %d, %d, %d\n", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    rx_odom_nrt->pose.position.x = msg->pose.position.x;
    rx_odom_nrt->pose.position.y = msg->pose.position.y;
    rx_odom_nrt->pose.position.z = msg->pose.position.z;
    rx_odom_nrt->pose.orientation.x = msg->pose.orientation.x;
    rx_odom_nrt->pose.orientation.y = msg->pose.orientation.y;
    rx_odom_nrt->pose.orientation.z = msg->pose.orientation.z;
    rx_odom_nrt->pose.orientation.w = msg->pose.orientation.w;
    // ROS_INFO("pose: %d, %d, %d\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    for(int i = 0; i<BUTTON_NUM; i++)
        rx_joy_nrt->buttons[i] = msg->buttons[i];

    // ROS_INFO("joy: %d, %d, %d, %d\n", msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);
    ret_joy = write(rx_joy_sockfd, (void *)rx_joy_nrt, BUFLEN_JOY);
    ret_odom = write(rx_odom_sockfd, (void *)rx_odom_nrt, BUFLEN_ODOM);
}

int main(int argc, char** argv) 
{
    if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_ODOM) < 0)
		fail("asprintf");
    rx_odom_sockfd = open(devname, O_RDWR);
	free(devname);
	if (rx_odom_sockfd < 0)
		fail("open");
    
    if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_JOY) < 0)
		fail("asprintf");
    rx_joy_sockfd = open(devname, O_RDWR);
	free(devname);
	if (rx_joy_sockfd < 0)
		fail("open");

    ros::init(argc, argv, "xddp_nrt");
    ros::NodeHandle nh_;

    ros::Publisher pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber subCmdVel = nh_.subscribe("/cmd_vel",1,cmdvelCallback);    
    ros::Subscriber subTwist = nh_.subscribe("/vive/right_controller/twist",1,twistCallback); 
    ros::Subscriber subPose = nh_.subscribe("/vive/right_controller/pose",1,poseCallback);        
    ros::Subscriber subJoy = nh_.subscribe("/vive/right_controller/joy",1,joyCallback);    
    
    
    ros::Rate loop_rate(100);


    ros::spin();
    loop_rate.sleep();
   
   
    return 0;
}
