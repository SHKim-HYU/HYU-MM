/*! 
 *  @file xddp_packet.h
 *  @brief header for xddp packet definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Jun. 13. 2024
 *  @Comm
 */

#pragma once

#define XDDP_PORT_CMD_VEL 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
#define XDDP_PORT_ODOM 1

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
}