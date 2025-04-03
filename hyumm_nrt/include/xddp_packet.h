/*! 
 *  @file xddp_packet.h
 *  @brief header for xddp packet definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Feb. 23. 2025
 *  @Comm
 */

#pragma once

/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
// Command
#define XDDP_PORT_CMD_VEL 0		// Command velocity for mobile
#define XDDP_PORT_CMD_ODOM 1	// Command odometry for target end-effector
#define XDDP_PORT_CMD_JOY 2		// Command Joystick button

// State
#define XDDP_PORT_ODOM 10		// State publish for mobile odometry
#define XDDP_PORT_SIM 11		// State publish for nominal mobile manipulator
#define XDDP_PORT_ACT 12		// State publish for actual mobile manipulator
#define XDDP_PORT_ADM 13		// State publish for admittance model

#define MM_DOF_NUM 6
#define BUTTON_NUM 4

namespace packet{

struct Vector3{
	double x;
	double y;
	double z;
};

struct Quaternion{
	double x;
	double y;
	double z;
	double w;
};

struct Pose{
	Vector3 position;
	Quaternion orientation;
};

struct Twist{
	Vector3 linear;
	Vector3 angular;
};

struct Odometry{
	Pose pose;
	Twist twist;	
};

struct JointState{
	double position[MM_DOF_NUM];
	double velocity[MM_DOF_NUM];
	double effort[MM_DOF_NUM];
};

struct Joy{
	int32_t buttons[BUTTON_NUM];
};
}