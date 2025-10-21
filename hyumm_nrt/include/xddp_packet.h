/*! 
 *  @file xddp_packet.h
 *  @brief header for xddp packet definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Jun. 13. 2024
 *  @Comm
 */

#pragma once

// Command
#define XDDP_PORT_CMD_INFO_1 0		// 
#define XDDP_PORT_CMD_INFO_2 1		// 
#define XDDP_PORT_CMD_INFO_3 2		// 
#define IDDP_PORT_CMD_INFO_1 3		// 
#define IDDP_PORT_CMD_INFO_2 4		// 
#define IDDP_PORT_CMD_INFO_3 5		// 

// State
#define XDDP_PORT_STATE_INFO_1 10	// 
#define XDDP_PORT_STATE_INFO_2 11	//
#define IDDP_PORT_STATE_INFO_1 12 	// 
#define IDDP_PORT_STATE_INFO_2 13	// 
#define IDDP_PORT_STATE_INFO_3 14	// 

// QT
#define XDDP_PORT_QT_RX0 20			// 
#define XDDP_PORT_QT_RX1 21			//
#define XDDP_PORT_QT_TX0 22			//
#define XDDP_PORT_QT_TX1 23			// 

<<<<<<< HEAD
#define MM_DOF_NUM 6
=======
>>>>>>> update
#define BUTTON_NUM 4

namespace packet{

struct Time{
	double global_time;
	double cycle_time;
	double exec_time;
};

template<int N>
struct CommState{
	std::array<uint16_t, N> status_word{};
  	std::array<uint16_t, N> control_word{};
	bool servo;
	bool halt;
	bool qstop;
	bool move;
	bool home;
	bool reset;
};


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

struct Accel{
	Vector3 linear;
	Vector3 angular;
};

struct Wrench{
	Vector3 force;
	Vector3 torque;
};

struct Odometry{
	Pose pose;
	Twist twist;	

};

struct Joy{
	int32_t buttons[BUTTON_NUM];
};

struct SingleRigidBody{
	Odometry odometry;
	Accel accel;
	Wrench wrench;
};

template<int N>
struct JointState{
	std::array<double, N> position;
	std::array<double, N> velocity;
	std::array<double, N> accel;
	std::array<double, N> effort;
};

template<int N>
struct RobotState{
	JointState<N> jointState;
	std::array<double, N>  effort_fric;
	std::array<double, N>  effort_ctrl;
	SingleRigidBody tcp;
	SingleRigidBody base;
};

template<int N>
struct RobotInfo{
	Time time;
	CommState<N> comm;
	RobotState<N> des;
	RobotState<N> act;
	RobotState<N> nom;
	RobotState<N> adm;
};

template<int N>
struct KUKAInfo{
	RobotInfo<N> kuka1;
	RobotInfo<N> kuka2;
};

template<int N>
struct KUKAState{
	RobotState<N> kuka1;
	RobotState<N> kuka2;
};


// EQM -> CDPR
template<int N>
struct IndyState{
	double gt_indy;
	bool rd_indy;
	JointState<N> eqm_sts;
};

// CDPR -> EQM
template<int N>
struct OptimalTorque{
	double gt_cdpr;
	bool rd_cdpr;
	JointState<N> cdpr_res;
};

}
