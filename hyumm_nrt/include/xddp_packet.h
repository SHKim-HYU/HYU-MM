/*! 
 *  @file xddp_packet.h
 *  @brief header for xddp packet definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Jun. 13. 2024
 *  @Comm
 */

#pragma once

#include <array>
#include <cstdint>

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

// Online-planning handshake between the RT controller and the OCS2 MPC (NRT).
//   RT  -> OCS2 : cmd  (start the front/back plan, or stop)
//   OCS2-> RT   : status + fault + seq (running/fault/done, fault code, liveness)
// Carried in RobotInfo.mpc on BOTH the RT->NRT state packet (cmd) and the
// NRT->RT desired packet (status), so a single packet type serves both directions.
// MUST stay byte-identical to KPOS_Foundation/include/Common/xddp_packet.h.
enum MpcCmd    : uint8_t { MPC_CMD_NONE = 0, MPC_CMD_START_FRONT = 1, MPC_CMD_START_BACK = 2, MPC_CMD_STOP = 3 };
enum MpcStatus : uint8_t { MPC_STAT_IDLE = 0, MPC_STAT_RUNNING = 1, MPC_STAT_FAULT = 2, MPC_STAT_DONE = 3 };
enum MpcFault  : uint16_t { MPC_FAULT_NONE = 0, MPC_FAULT_COLLISION = 1, MPC_FAULT_MPC_FAIL = 2, MPC_FAULT_NO_PLAN = 3 };

struct MpcHandshake{
	uint8_t  cmd{0};     // RT  -> OCS2 : MpcCmd
	uint8_t  status{0};  // OCS2-> RT   : MpcStatus
	uint16_t fault{0};   // OCS2-> RT   : MpcFault (0 = none)
	uint32_t seq{0};     // OCS2-> RT   : setpoint sequence (increments each cycle; staleness/liveness)
};

template<int N>
struct RobotInfo{
	Time time;
	CommState<N> comm;
	RobotState<N> des;
	RobotState<N> act;
	RobotState<N> nom;
	RobotState<N> adm;
	MpcHandshake mpc;   // OCS2 <-> RT online-planning handshake (appended; existing offsets unchanged)
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
