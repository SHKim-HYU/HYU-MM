#include "multi_hyumm_xddp_node.h"

static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void readOdomData(ros::Publisher& pubOdometry)
{
	if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_ODOM) < 0)
		fail("asprintf");

	odom_sockfd = open(devname, O_RDONLY);
	free(devname);
	if (odom_sockfd < 0)
		fail("open");

    while (ros::ok())
    {
        /* Get the next message from realtime_thread. */
		ret = read(odom_sockfd, (void *)odom_nrt, BUFLEN_ODOM); 

        if(ret>0)
        {
            odom_msg.header.stamp = ros::Time::now();
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

            pubOdometry.publish(odom_msg);
        }
    }
    close(odom_sockfd);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
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
    if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_VEL) < 0)
		fail("asprintf");

	cmd_vel_sockfd = open(devname, O_RDWR);
	free(devname);
	if (cmd_vel_sockfd < 0)
		fail("open");


    ros::init(argc, argv, "xddp_nrt");
    ros::NodeHandle nh_;

    ros::Publisher pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber subTwist = nh_.subscribe("/cmd_vel",1,twistCallback);    
    

    std::thread odom_thread(readOdomData, std::ref(pubOdometry));

    ros::Rate loop_rate(1000);


    ros::spin();
    loop_rate.sleep();
   
    odom_thread.join();

    return 0;
}
