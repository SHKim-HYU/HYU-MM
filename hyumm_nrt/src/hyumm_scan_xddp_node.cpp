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
static const double OFFSET_REAR_X = -0.395;
static const double OFFSET_REAR_Y = -0.235;
static const double OFFSET_FRONT_X = 0.420;   // TODO: measure actual value
static const double OFFSET_FRONT_Y = 0.235;  // TODO: measure actual value

// Angle of rear-to-front line in base frame (for heading correction when both active)
static const double TRACKER_LINE_ANGLE = atan2(OFFSET_FRONT_Y - OFFSET_REAR_Y,
                                                OFFSET_FRONT_X - OFFSET_REAR_X);

// --- Occlusion timeout (seconds) ---
static const double OCCLUSION_TIMEOUT = 0.1;

// --- Tracker data ---
static geometry_msgs::PoseStamped fixed_pose, mobile_rear_pose, mobile_front_pose;
static geometry_msgs::TwistStamped mobile_rear_twist, mobile_front_twist;
static bool fixed_received = false;
static ros::Time mobile_rear_last_time, mobile_front_last_time;
static bool mobile_rear_ever_received = false, mobile_front_ever_received = false;

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

void mobileRearPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	mobile_rear_pose = *msg;
	mobile_rear_last_time = ros::Time::now();
	mobile_rear_ever_received = true;
}

void mobileRearTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	mobile_rear_twist = *msg;
}

void mobileFrontPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	mobile_front_pose = *msg;
	mobile_front_last_time = ros::Time::now();
	mobile_front_ever_received = true;
}

void mobileFrontTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	mobile_front_twist = *msg;
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
	if(!fixed_received) return;

	// --- Check tracker occlusion ---
	ros::Time now = ros::Time::now();
	bool rear_active = mobile_rear_ever_received &&
	                   (now - mobile_rear_last_time).toSec() < OCCLUSION_TIMEOUT;
	bool front_active = mobile_front_ever_received &&
	                    (now - mobile_front_last_time).toSec() < OCCLUSION_TIMEOUT;

	if(!rear_active && !front_active) return;

	// --- Calibration transform ---
	tf::Transform T_fixed_vive = poseToTransform(fixed_pose.pose);
	tf::Transform T_fixed_ref(tf::Quaternion(0, 0, 0, 1),
	                           tf::Vector3(REF_X, REF_Y, REF_Z));
	tf::Transform T_calib = T_fixed_ref * T_fixed_vive.inverse();

	double x, y, raw_yaw;
	tf::Transform T_active_ref;       // tracker-in-ref for twist computation
	double active_offset_x, active_offset_y;
	const geometry_msgs::TwistStamped* active_twist;

	if(rear_active && front_active) {
		// ====== BOTH TRACKERS: fused mode ======
		tf::Transform T_rear_ref  = T_calib * poseToTransform(mobile_rear_pose.pose);
		tf::Transform T_front_ref = T_calib * poseToTransform(mobile_front_pose.pose);

		double rear_x  = T_rear_ref.getOrigin().x();
		double rear_y  = T_rear_ref.getOrigin().y();
		double front_x = T_front_ref.getOrigin().x();
		double front_y = T_front_ref.getOrigin().y();

		// Heading from rear-to-front vector, corrected for tracker line angle in base frame
		raw_yaw = atan2(front_y - rear_y, front_x - rear_x) - TRACKER_LINE_ANGLE;
		raw_yaw = normalizeAngle(raw_yaw);

		double cos_yaw = cos(raw_yaw);
		double sin_yaw = sin(raw_yaw);

		// Base center estimated from each tracker (subtract rotated offset)
		double base_x_rear  = rear_x  - (cos_yaw * OFFSET_REAR_X  - sin_yaw * OFFSET_REAR_Y);
		double base_y_rear  = rear_y  - (sin_yaw * OFFSET_REAR_X  + cos_yaw * OFFSET_REAR_Y);
		double base_x_front = front_x - (cos_yaw * OFFSET_FRONT_X - sin_yaw * OFFSET_FRONT_Y);
		double base_y_front = front_y - (sin_yaw * OFFSET_FRONT_X + cos_yaw * OFFSET_FRONT_Y);

		// Average both estimates
		x = (base_x_rear + base_x_front) / 2.0;
		y = (base_y_rear + base_y_front) / 2.0;

		// Use rear tracker for twist computation
		T_active_ref = T_rear_ref;
		active_offset_x = OFFSET_REAR_X;
		active_offset_y = OFFSET_REAR_Y;
		active_twist = &mobile_rear_twist;

	} else if(rear_active) {
		// ====== REAR ONLY (front occluded) ======
		tf::Transform T_rear_ref = T_calib * poseToTransform(mobile_rear_pose.pose);
		tf::Transform T_rear_offset(tf::Quaternion(0, 0, 0, 1),
		                             tf::Vector3(OFFSET_REAR_X, OFFSET_REAR_Y, 0));
		tf::Transform T_base = T_rear_ref * T_rear_offset.inverse();

		x = T_base.getOrigin().x();
		y = T_base.getOrigin().y();

		double roll, pitch;
		T_base.getBasis().getRPY(roll, pitch, raw_yaw);

		T_active_ref = T_rear_ref;
		active_offset_x = OFFSET_REAR_X;
		active_offset_y = OFFSET_REAR_Y;
		active_twist = &mobile_rear_twist;

	} else {
		// ====== FRONT ONLY (rear occluded) ======
		tf::Transform T_front_ref = T_calib * poseToTransform(mobile_front_pose.pose);
		tf::Transform T_front_offset(tf::Quaternion(0, 0, 0, 1),
		                              tf::Vector3(OFFSET_FRONT_X, OFFSET_FRONT_Y, 0));
		tf::Transform T_base = T_front_ref * T_front_offset.inverse();

		x = T_base.getOrigin().x();
		y = T_base.getOrigin().y();

		double roll, pitch;
		T_base.getBasis().getRPY(roll, pitch, raw_yaw);

		T_active_ref = T_front_ref;
		active_offset_x = OFFSET_FRONT_X;
		active_offset_y = OFFSET_FRONT_Y;
		active_twist = &mobile_front_twist;
	}

	// --- Multi-turn yaw unwrapping ---
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

	// --- Twist: transform active tracker velocity to base center velocity in ref frame ---
	tf::Vector3 v_lin_body(active_twist->twist.linear.x,
	                       active_twist->twist.linear.y,
	                       active_twist->twist.linear.z);
	tf::Vector3 v_ang_body(active_twist->twist.angular.x,
	                       active_twist->twist.angular.y,
	                       active_twist->twist.angular.z);

	// Rotate twist from tracker body frame to reference frame
	tf::Matrix3x3 R_ref = T_active_ref.getBasis();
	tf::Vector3 v_tracker_ref = R_ref * v_lin_body;
	tf::Vector3 w_ref         = R_ref * v_ang_body;

	// Offset correction: v_base = v_tracker - w x r_offset_ref
	double cos_yaw_cur = cos(raw_yaw);
	double sin_yaw_cur = sin(raw_yaw);
	tf::Vector3 r_offset_ref(cos_yaw_cur * active_offset_x - sin_yaw_cur * active_offset_y,
	                          sin_yaw_cur * active_offset_x + cos_yaw_cur * active_offset_y,
	                          0);
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

	// Fixed tracker
	ros::Subscriber sub_fixed_pose = nh.subscribe("/vive/tracker_fixed/pose", 1, fixedPoseCallback);

	// Rear tracker
	ros::Subscriber sub_mobile_rear_pose  = nh.subscribe("/vive/tracker_mobile_rear/pose",  1, mobileRearPoseCallback);
	ros::Subscriber sub_mobile_rear_twist = nh.subscribe("/vive/tracker_mobile_rear/twist", 1, mobileRearTwistCallback);

	// Front tracker
	ros::Subscriber sub_mobile_front_pose  = nh.subscribe("/vive/tracker_mobile_front/pose",  1, mobileFrontPoseCallback);
	ros::Subscriber sub_mobile_front_twist = nh.subscribe("/vive/tracker_mobile_front/twist", 1, mobileFrontTwistCallback);

	// cmd_vel
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
