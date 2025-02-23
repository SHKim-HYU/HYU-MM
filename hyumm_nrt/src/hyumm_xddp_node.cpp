#include "hyumm_xddp_node.h"

static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void readOdomData(ros::Publisher& pubOdometry)
{
	if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_ODOM) < 0)
		fail("asprintf");

	tx_odom_sockfd = open(devname, O_RDONLY);
	free(devname);
	if (tx_odom_sockfd < 0)
		fail("open");

    while (ros::ok())
    {
        /* Get the next message from realtime_thread. */
		ret = read(tx_odom_sockfd, (void *)tx_odom_nrt, BUFLEN_ODOM); 

        if(ret>0)
        {
            tx_odom_msg.header.stamp = ros::Time::now();
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

            pubOdometry.publish(tx_odom_msg);
        }
    }
    close(tx_odom_sockfd);
}

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    rx_cmd_vel_nrt->linear.x = msg->linear.x;
    rx_cmd_vel_nrt->linear.y = msg->linear.y;
    rx_cmd_vel_nrt->linear.z = msg->linear.z;
    rx_cmd_vel_nrt->angular.x = msg->angular.x;
    rx_cmd_vel_nrt->angular.y = msg->angular.y;
    rx_cmd_vel_nrt->angular.z = msg->angular.z;

    ret_twist = write(rx_cmd_vel_sockfd, (void *)rx_cmd_vel_nrt, BUFLEN_TWIST);
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    rx_odom_nrt->twist.linear.x = msg->twist.linear.x;
    rx_odom_nrt->twist.linear.y = msg->twist.linear.y;
    rx_odom_nrt->twist.linear.z = msg->twist.linear.z;
    rx_odom_nrt->twist.angular.x = msg->twist.angular.x;
    rx_odom_nrt->twist.angular.y = msg->twist.angular.y;
    rx_odom_nrt->twist.angular.z = msg->twist.angular.z;

}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    rx_odom_nrt->pose.position.x = msg->pose.position.x;
    rx_odom_nrt->pose.position.y = msg->pose.position.y;
    rx_odom_nrt->pose.position.z = msg->pose.position.z;
    rx_odom_nrt->pose.orientation.x = msg->pose.orientation.x;
    rx_odom_nrt->pose.orientation.y = msg->pose.orientation.y;
    rx_odom_nrt->pose.orientation.z = msg->pose.orientation.z;
    rx_odom_nrt->pose.orientation.w = msg->pose.orientation.w;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    for(int i = 0; i<BUTTON_NUM; i++)
        rx_joy_nrt->buttons[i] = msg->buttons[i];

    // printf("joy: %d, %d, %d, %d\n", msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);
    ret_joy = write(rx_joy_sockfd, (void *)rx_joy_nrt, BUFLEN_JOY);
    ret_odom = write(rx_odom_sockfd, (void *)rx_odom_nrt, BUFLEN_ODOM);
}

int main(int argc, char** argv) 
{
    if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_VEL) < 0)
		fail("asprintf");
    rx_cmd_vel_sockfd = open(devname, O_RDWR);
	free(devname);
	if (rx_cmd_vel_sockfd < 0)
		fail("open");

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
    

    std::thread odom_thread(readOdomData, std::ref(pubOdometry));
    
    ros::Rate loop_rate(1000);


    ros::spin();
    loop_rate.sleep();
   
    odom_thread.join();

    return 0;
}
