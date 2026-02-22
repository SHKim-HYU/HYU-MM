#include "ros/ros.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "xddp_ros.h"

// --- XDDP ---
static int xddp_fd = -1;
static int xddp_cmdvel_fd = -1;
static const size_t BUFLEN_ODOM = sizeof(packet::Odometry);
static const size_t BUFLEN_TWIST = sizeof(packet::Twist);
static packet::Odometry tx_msg;
static packet::Twist tx_cmdvel_msg;

// --- Calibration parameters ---
// Fixed tracker known position in world frame
static const double REF_X = 1.5;
static const double REF_Y = 0.3;
static const double REF_Z = 0.8;
// Fixed tracker orientation: identity (0,0,0,1)

// Tracker offset from mobile base center (in base frame)
static const double OFFSET_X = -0.395;
static const double OFFSET_Y = -0.235;

// --- Tracker data ---
static geometry_msgs::PoseStamped fixed_pose, mobile_pose;
static geometry_msgs::TwistStamped mobile_twist;
static bool fixed_received = false, mobile_received = false;

// --- Multi-turn yaw tracking ---
static double accumulated_yaw = 0.0;
static double prev_raw_yaw = 0.0;
static bool yaw_initialized = false;

static double normalizeAngle(double a) {
	while(a >  M_PI) a -= 2.0 * M_PI;
	while(a < -M_PI) a += 2.0 * M_PI;
	return a;
}

// Helper: geometry_msgs::Pose -> tf::Transform
static tf::Transform poseToTransform(const geometry_msgs::Pose& p) {
	return tf::Transform(
		tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
		tf::Vector3(p.position.x, p.position.y, p.position.z));
}

void fixedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	fixed_pose = *msg;
	fixed_received = true;
}

void mobilePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	mobile_pose = *msg;
	mobile_received = true;
}

void mobileTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	mobile_twist = *msg;
}

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	tx_cmdvel_msg.linear.x = msg->linear.x;
	tx_cmdvel_msg.linear.y = msg->linear.y;
	tx_cmdvel_msg.linear.z = msg->linear.z;
	tx_cmdvel_msg.angular.x = msg->angular.x;
	tx_cmdvel_msg.angular.y = msg->angular.y;
	tx_cmdvel_msg.angular.z = msg->angular.z;

	write(xddp_cmdvel_fd, &tx_cmdvel_msg, BUFLEN_TWIST);
}

void computeAndSend() {
	if(!fixed_received || !mobile_received) return;

	// --- Build tf::Transforms ---
	tf::Transform T_fixed_vive  = poseToTransform(fixed_pose.pose);
	tf::Transform T_mobile_vive = poseToTransform(mobile_pose.pose);

	tf::Transform T_fixed_ref(tf::Quaternion(0, 0, 0, 1),
	                           tf::Vector3(REF_X, REF_Y, REF_Z));

	tf::Transform T_offset(tf::Quaternion(0, 0, 0, 1),
	                        tf::Vector3(OFFSET_X, OFFSET_Y, 0));

	// T_calib = T_fixed_ref * inv(T_fixed_vive)
	tf::Transform T_calib = T_fixed_ref * T_fixed_vive.inverse();

	// T_mobile_ref = T_calib * T_mobile_vive
	tf::Transform T_mobile_ref = T_calib * T_mobile_vive;

	// T_base = T_mobile_ref * inv(T_offset)
	tf::Transform T_base = T_mobile_ref * T_offset.inverse();

	// --- Extract x, y ---
	double x = T_base.getOrigin().x();
	double y = T_base.getOrigin().y();

	// --- Multi-turn yaw unwrapping ---
	double roll, pitch, raw_yaw;
	T_base.getBasis().getRPY(roll, pitch, raw_yaw);

	if(!yaw_initialized) {
		accumulated_yaw = raw_yaw;
		prev_raw_yaw = raw_yaw;
		yaw_initialized = true;
	} else {
		double delta = normalizeAngle(raw_yaw - prev_raw_yaw);
		accumulated_yaw += delta;
		prev_raw_yaw = raw_yaw;
	}

	// --- Fill odom packet ---
	memset(&tx_msg, 0, sizeof(tx_msg));
	tx_msg.pose.position.x = x;
	tx_msg.pose.position.y = y;
	tx_msg.pose.position.z = accumulated_yaw;  // continuous yaw (multi-turn)

	tx_msg.pose.orientation.x = 0.0;
	tx_msg.pose.orientation.y = 0.0;
	tx_msg.pose.orientation.z = sin(accumulated_yaw / 2.0);
	tx_msg.pose.orientation.w = cos(accumulated_yaw / 2.0);

	// --- Twist: transform tracker velocity to base center velocity in ref frame ---
	tf::Vector3 v_lin_body(mobile_twist.twist.linear.x,
	                       mobile_twist.twist.linear.y,
	                       mobile_twist.twist.linear.z);
	tf::Vector3 v_ang_body(mobile_twist.twist.angular.x,
	                       mobile_twist.twist.angular.y,
	                       mobile_twist.twist.angular.z);

	// Rotate twist from tracker body frame to reference frame
	tf::Matrix3x3 R_mobile_ref = T_mobile_ref.getBasis();
	tf::Vector3 v_tracker_ref = R_mobile_ref * v_lin_body;
	tf::Vector3 w_ref         = R_mobile_ref * v_ang_body;

	// Offset correction: v_base = v_tracker - ω × r_offset_ref
	// r_offset is vector from base center to tracker in reference frame
	tf::Matrix3x3 R_base = T_base.getBasis();
	tf::Vector3 r_offset_ref = R_base * tf::Vector3(OFFSET_X, OFFSET_Y, 0);
	tf::Vector3 v_base_ref = v_tracker_ref - w_ref.cross(r_offset_ref);

	tx_msg.twist.linear.x  = v_base_ref.x();
	tx_msg.twist.linear.y  = v_base_ref.y();
	tx_msg.twist.linear.z  = 0.0;
	tx_msg.twist.angular.x = 0.0;
	tx_msg.twist.angular.y = 0.0;
	tx_msg.twist.angular.z = w_ref.z();

	// Send via XDDP
	write(xddp_fd, &tx_msg, BUFLEN_ODOM);
}

static void fail(const char *reason) {
	perror(reason);
	exit(EXIT_FAILURE);
}

int main(int argc, char** argv)
{
	// Open XDDP port for writing odom to RT
	char *devname;
	if(asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_INFO_2) < 0)
		fail("asprintf");
	xddp_fd = open(devname, O_RDWR);
	free(devname);
	if(xddp_fd < 0)
		fail("open xddp_fd");

	// Open XDDP port for writing cmd_vel to RT
	if(asprintf(&devname, "/dev/rtp%d", XDDP_PORT_CMD_INFO_1) < 0)
		fail("asprintf");
	xddp_cmdvel_fd = open(devname, O_RDWR);
	free(devname);
	if(xddp_cmdvel_fd < 0)
		fail("open xddp_cmdvel_fd");

	ros::init(argc, argv, "hyumm_scan_xddp_node");
	ros::NodeHandle nh;

	ros::Subscriber sub_fixed_pose  = nh.subscribe("/vive/tracker_fixed/pose",  1, fixedPoseCallback);
	ros::Subscriber sub_mobile_pose = nh.subscribe("/vive/tracker_mobile/pose", 1, mobilePoseCallback);
	ros::Subscriber sub_mobile_twist = nh.subscribe("/vive/tracker_mobile/twist", 1, mobileTwistCallback);
	ros::Subscriber sub_cmd_vel = nh.subscribe("/cmd_vel", 1, cmdvelCallback);

	ros::Rate rate(100);
	while(ros::ok()) {
		ros::spinOnce();
		computeAndSend();
		rate.sleep();
	}

	close(xddp_fd);
	close(xddp_cmdvel_fd);
	return 0;
}
