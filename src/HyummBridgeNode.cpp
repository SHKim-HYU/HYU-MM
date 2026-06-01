// HyummBridgeNode -- the REAL MRT loop (ROS2 Galactic).
//
// Replaces hyumm_ocs2's MRT_ROS_Dummy_Loop (which forward-SIMULATES the state)
// with a loop driven by MEASURED state, and emits the MPC-optimal desired joints
// to the Xenomai RT controller over XDDP. Owns an ocs2::MRT_ROS_Interface (the
// same object HyummDummyMrtNode creates) and every cycle:
//   1. assembles a measured 9-D SystemObservation:
//        state = [ base_x, base_y, base_yaw (from vive) | q0..q5 (arm, /joint_states) ]
//   2. setCurrentObservation(obs)  -> MPC re-solves against the TRUE state
//   3. updatePolicy() + evaluatePolicy(t, measured_state) -> desired xd / ud
//   4. writes packet::RobotInfo<9>.des (position=xd, velocity=ud) to /dev/rtpN
//      via the self-healing XddpLink (byte-identical wire format to the RT side).
//
// Architecture: OPTION A (MVP whole-body wiring) -- the existing 9-DOF hyumm_ocs2
// MPC is reused UNCHANGED. The decoupled 6-DOF refactor is Phase 4.
//
// SAFETY: send_desired defaults to false, so nothing is written to RT until the
// operator confirms the RT-side contract (port, task name, packet, setpoint
// semantics -- see config/hyumm_bridge.yaml) and enables it.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "hyumm_ocs2/HyummInterface.h"
#include "hyumm_ocs2/definitions.h"
#include "hyumm_ocs2/package_path.h"
#include "hyumm_bridge/CollisionMonitor.h"
#include "xddp_ros.h"   // XddpLink (self-healing /dev/rtpN) + packet:: schema (from hyumm_nrt)

namespace {
constexpr int MM_DOF = 9;   // matches packet::RobotInfo<9> / hyumm_ocs2 STATE_DIM
}

class HyummBridgeNode : public rclcpp::Node {
 public:
  HyummBridgeNode() : rclcpp::Node("hyumm_bridge_node") {
    const std::string pkg = hyumm_ocs2::getPackagePath();
    task_file_ = declare_parameter<std::string>("taskFile", pkg + "/config/task.info");
    urdf_file_ = declare_parameter<std::string>("urdfFile", pkg + "/urdf/hyumm_scan.urdf");
    lib_folder_ = declare_parameter<std::string>("libFolder", "/tmp/hyumm_ocs2");

    base_pose_topic_ = declare_parameter<std::string>(
        "base_pose_topic", "/vive_world/base_link/pose");
    joint_states_topic_ = declare_parameter<std::string>(
        "joint_states_topic", "/joint_states");
    arm_joint_names_ = declare_parameter<std::vector<std::string>>(
        "arm_joint_names", {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"});
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", 100.0);

    // XDDP desired channel (NRT -> RT). All RT-contract values are CONFIRM-with-RT.
    send_desired_ = declare_parameter<bool>("send_desired", false);
    desired_port_ = declare_parameter<int>("xddp_desired_port", 2);  // XDDP_PORT_CMD_INFO_3 -> /dev/rtp2
    rt_task_desired_ = declare_parameter<std::string>("rt_task_desired", "rx_desired_joints_task");
    watchdog_period_ = declare_parameter<double>("watchdog_period", 0.5);
    // "state" = use plan state xd as the desired position; "integrate" = integral of ud.
    desired_position_source_ = declare_parameter<std::string>("desired_position_source", "state");
    // Whether the base velocity (input[0..2]) is also sent to RT (des[0..2]); if
    // false the base stays on its own follower and only arm des[3..8] is meaningful.
    command_base_ = declare_parameter<bool>("command_base", true);

    // Safety clamps on the emitted velocity command (per-DOF magnitude).
    max_base_vel_ = declare_parameter<double>("max_base_vel", 1.0);   // m/s, rad/s
    max_arm_vel_ = declare_parameter<double>("max_arm_vel", 2.0);     // rad/s

    // --- Phase-clock collision-stop ---
    // When enabled, a binary self-collision monitor (built in main) gates the
    // emitted desired velocity by a rate in [0,1]: rate ramps to 0 on a current/
    // imminent collision (freeze) and back to 1 when clear (resume). The desired
    // position is integrated from the rate-scaled velocity so position and
    // velocity stay consistent through the stop. Default off (safety).
    collision_stop_ = declare_parameter<bool>("collision_stop", false);
    collision_lookahead_ = declare_parameter<int>("collision_lookahead", 10);
    rate_ramp_up_ = declare_parameter<double>("rate_ramp_up", 0.02);     // per cycle (resume slowly)
    rate_ramp_down_ = declare_parameter<double>("rate_ramp_down", 0.20); // per cycle (stop fast)

    arm_q_.assign(arm_joint_names_.size(), 0.0);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        base_pose_topic_, 10,
        std::bind(&HyummBridgeNode::poseCb, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        std::bind(&HyummBridgeNode::jointCb, this, std::placeholders::_1));

    desired_link_.configure("des", desired_port_, rt_task_desired_, send_desired_);
    desired_link_.setLogger(get_logger());
    desired_link_.poll();
    watchdog_ = create_wall_timer(
        std::chrono::duration<double>(watchdog_period_),
        [this]() { desired_link_.poll(); });

    RCLCPP_INFO(get_logger(),
        "hyumm_bridge: base<-%s arm<-%s ; send_desired=%s (port /dev/rtp%d task '%s')",
        base_pose_topic_.c_str(), joint_states_topic_.c_str(),
        send_desired_ ? "true" : "false", desired_port_, rt_task_desired_.c_str());
  }

  // --- accessors used by main() ---
  const std::string& taskFile() const { return task_file_; }
  const std::string& urdfFile() const { return urdf_file_; }
  const std::string& libFolder() const { return lib_folder_; }
  double loopRateHz() const { return loop_rate_hz_; }
  bool haveBase() const { return have_base_; }
  bool haveArm() const { return have_arm_; }
  bool collisionStopEnabled() const { return collision_stop_; }
  int collisionLookahead() const { return collision_lookahead_; }
  double rate() const { return rate_; }

  // Phase clock: ramp the rate toward 0 on hazard (stop), toward 1 when clear
  // (resume), with asymmetric ramps (stop fast, resume slow = hysteresis).
  void updatePhaseClock(bool hazard) {
    if (!collision_stop_) { rate_ = 1.0; return; }
    if (hazard) rate_ = std::max(0.0, rate_ - rate_ramp_down_);
    else        rate_ = std::min(1.0, rate_ + rate_ramp_up_);
  }

  // Overwrite state[0..8] with the latest measurements (leaves arm/base untouched
  // if not yet received, so the seed initial state is used until data arrives).
  void fillMeasuredState(ocs2::vector_t& state) const {
    if (have_base_) {
      state(hyumm_ocs2::state_idx::BASE_X) = base_x_;
      state(hyumm_ocs2::state_idx::BASE_Y) = base_y_;
      state(hyumm_ocs2::state_idx::BASE_PSI) = base_yaw_;  // continuous (unwrapped)
    }
    if (have_arm_) {
      const size_t n = std::min<size_t>(arm_q_.size(), hyumm_ocs2::ARM_DIM);
      for (size_t i = 0; i < n; ++i)
        state(hyumm_ocs2::state_idx::ARM_Q0 + i) = arm_q_[i];
    }
  }

  // Emit the MPC-optimal desired joints to RT over XDDP (no-op while link DOWN /
  // send_desired=false).
  void sendDesired(const ocs2::vector_t& xd, const ocs2::vector_t& ud, double dt) {
    if (!send_desired_) return;
    if (!int_init_) {
      for (int i = 0; i < MM_DOF; ++i) des_pos_int_[i] = xd(i);
      int_init_ = true;
    }
    // Under collision-stop the position MUST track the rate-scaled velocity
    // (else a frozen velocity with a still-advancing setpoint would fight).
    const bool use_int = collision_stop_ || desired_position_source_ == "integrate";

    packet::RobotInfo<MM_DOF> pkt{};   // {} zero-inits all sub-structs
    for (int i = 0; i < MM_DOF; ++i) {
      const bool isBase = (i < static_cast<int>(hyumm_ocs2::BASE_INPUT_DIM));
      double vel = ud(i);
      const double lim = isBase ? max_base_vel_ : max_arm_vel_;
      vel = std::max(-lim, std::min(lim, vel));   // safety clamp
      if (isBase && !command_base_) vel = 0.0;
      vel *= rate_;                               // phase-clock gate (1.0 unless stopping)

      double pos;
      if (use_int) { des_pos_int_[i] += vel * dt; pos = des_pos_int_[i]; }
      else { pos = xd(i); }
      pkt.des.jointState.position[i] = pos;
      pkt.des.jointState.velocity[i] = vel;
      pkt.des.jointState.accel[i] = 0.0;   // OCS2 outputs velocity; no accel reference
    }
    desired_link_.tryWrite(pkt);
  }

 private:
  void poseCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
    base_x_ = m->pose.position.x;
    base_y_ = m->pose.position.y;
    // planar yaw from the (z-only) quaternion
    const double qw = m->pose.orientation.w, qx = m->pose.orientation.x;
    const double qy = m->pose.orientation.y, qz = m->pose.orientation.z;
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
    // Multi-turn unwrap so BASE_PSI is continuous (the published pose quaternion
    // wraps to [-pi,pi]; a wrapped psi would corrupt the MPC near +-pi).
    if (!yaw_init_) { acc_yaw_ = yaw; prev_yaw_ = yaw; yaw_init_ = true; }
    else {
      double d = yaw - prev_yaw_;
      while (d > M_PI) d -= 2.0 * M_PI;
      while (d < -M_PI) d += 2.0 * M_PI;
      acc_yaw_ += d;
      prev_yaw_ = yaw;
    }
    base_yaw_ = acc_yaw_;
    have_base_ = true;
  }

  void jointCb(sensor_msgs::msg::JointState::ConstSharedPtr m) {
    size_t found = 0;
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      for (size_t j = 0; j < m->name.size(); ++j) {
        if (m->name[j] == arm_joint_names_[i] && j < m->position.size()) {
          arm_q_[i] = m->position[j];
          ++found;
          break;
        }
      }
    }
    if (found == arm_joint_names_.size()) have_arm_ = true;
  }

  // params / config
  std::string task_file_, urdf_file_, lib_folder_;
  std::string base_pose_topic_, joint_states_topic_;
  std::vector<std::string> arm_joint_names_;
  double loop_rate_hz_ = 100.0;
  bool send_desired_ = false;
  int desired_port_ = 2;
  std::string rt_task_desired_;
  double watchdog_period_ = 0.5;
  std::string desired_position_source_ = "state";
  bool command_base_ = true;
  double max_base_vel_ = 1.0, max_arm_vel_ = 2.0;

  // phase-clock collision-stop
  bool collision_stop_ = false;
  int collision_lookahead_ = 10;
  double rate_ramp_up_ = 0.02, rate_ramp_down_ = 0.20;
  double rate_ = 1.0;       // phase-clock rate in [0,1]
  bool int_init_ = false;   // desired-position integrator seeded?

  // measured state cache
  double base_x_ = 0.0, base_y_ = 0.0, base_yaw_ = 0.0;
  std::vector<double> arm_q_;
  bool have_base_ = false, have_arm_ = false;
  bool yaw_init_ = false;
  double acc_yaw_ = 0.0, prev_yaw_ = 0.0;
  double des_pos_int_[MM_DOF] = {0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  XddpLink desired_link_;
};

int main(int argc, char** argv) {
  const std::string robotName = "hyumm";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HyummBridgeNode>();

  RCLCPP_INFO(node->get_logger(), "Loading task file: %s", node->taskFile().c_str());
  RCLCPP_INFO(node->get_logger(), "Loading URDF file: %s", node->urdfFile().c_str());
  RCLCPP_INFO(node->get_logger(), "Loading library folder: %s", node->libFolder().c_str());

  // Build the OCS2 interface (same as HyummDummyMrtNode) and MRT.
  hyumm_ocs2::HyummInterface interface(
      node->taskFile(), node->urdfFile(), node->libFolder(), true);

  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());

  // Seed observation + target (until the first MPC policy / measurements arrive).
  ocs2::SystemObservation obs;
  obs.state = interface.getInitialState();
  obs.input.setZero(hyumm_ocs2::INPUT_DIM);
  obs.time = 0.0;
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(hyumm_ocs2::INPUT_DIM);
  const ocs2::TargetTrajectories initTarget({obs.time}, {obs.state}, {zeroInput});

  // launchNodes() FIRST (creates the policy sub, observation pub and the MPC
  // reset service client on `node`), THEN resetMpcNode() which blocks until the
  // MPC reset service answers -- so hyumm_mpc_node must be running.
  mrt.launchNodes(node);
  RCLCPP_INFO(node->get_logger(), "Resetting MPC (waiting for hyumm_mpc_node)...");
  mrt.resetMpcNode(initTarget);

  // Binary self-collision monitor for the phase-clock (built from task.info
  // collisionLinkPairs; independent of the SQP self-collision constraint).
  std::unique_ptr<hyumm_bridge::CollisionMonitor> monitor;
  if (node->collisionStopEnabled()) {
    monitor = hyumm_bridge::CollisionMonitor::loadFromTaskFile(
        node->taskFile(), interface.getPinocchioInterface());
    if (monitor) {
      RCLCPP_INFO(node->get_logger(),
                  "collision-stop ENABLED: %zu pairs, min distance %.3f m",
                  monitor->numCollisionPairs(), monitor->getMinimumDistance());
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "collision_stop set but no collisionLinkPairs in task.info -- disabled");
    }
  }

  const double dt = 1.0 / node->loopRateHz();
  rclcpp::Rate rate(node->loopRateHz());
  double t = 0.0;
  bool warned_wait = false;
  bool got_policy = false;

  RCLCPP_INFO(node->get_logger(), "HYUMM bridge (real MRT loop) starting at %.0f Hz", node->loopRateHz());
  while (rclcpp::ok()) {
    mrt.spinMRT();   // services MPC policy callback AND our pose/joint_states/watchdog

    // 1) assemble the MEASURED 9-D observation
    obs.time = t;
    node->fillMeasuredState(obs.state);

    // 2) feedback to MPC
    mrt.setCurrentObservation(obs);

    // 3) swap in the latest policy
    mrt.updatePolicy();

    // 4) if a plan exists, compute desired joints AT THE MEASURED STATE and emit
    if (mrt.initialPolicyReceived()) {
      if (!got_policy) {
        RCLCPP_INFO(node->get_logger(),
                    "first MPC policy received -- closed-loop running");
        got_policy = true;
      }
      ocs2::vector_t xd, ud;
      size_t mode = 0;
      mrt.evaluatePolicy(t, obs.state, xd, ud, mode);

      // Phase-clock collision-stop: hazard = current OR imminent (planned) collision.
      bool hazard = false;
      double min_dist = 0.0;
      if (monitor) {
        min_dist = monitor->minDistance(obs.state);
        hazard = min_dist < monitor->getMinimumDistance();
        if (!hazard) {  // predictive lookahead over the planned state trajectory
          const auto& sol = mrt.getPolicy();
          const int N = std::min<int>(node->collisionLookahead(),
                                      static_cast<int>(sol.stateTrajectory_.size()));
          for (int k = 0; k < N; ++k) {
            if (monitor->isInCollision(sol.stateTrajectory_[k])) { hazard = true; break; }
          }
        }
      }
      node->updatePhaseClock(hazard);

      node->sendDesired(xd, ud, dt);
      obs.input = ud;   // report the applied input back to MPC next cycle
      // periodic heartbeat: measured base + desired base/arm velocity + phase clock
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
          "meas base[%.3f %.3f %.3f] meas?=%d/%d  des dq0=%.2f  rate=%.2f%s minDist=%.3f",
          obs.state(0), obs.state(1), obs.state(2),
          node->haveBase(), node->haveArm(), ud(3),
          node->rate(), hazard ? " HAZARD" : "", min_dist);
    } else if (!warned_wait) {
      RCLCPP_WARN(node->get_logger(),
                  "waiting for first MPC policy (is hyumm_mpc_node running?)");
      warned_wait = true;
    }

    t += dt;
    rate.sleep();
  }

  mrt.shutdownNodes();
  rclcpp::shutdown();
  return 0;
}
