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
#include "hyumm_ocs2/reference/CsvTargetLoader.h"
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
    // Default false: with model_settings.controlBase=false the MPC FREEZES the base
    // (u_base=0) and plans only the arm, so OCS2 has no meaningful base command --
    // the base follows its own trajectory follower and OCS2 compensates via the arm.
    // Whole-body: the MPC plans the base too, so its base command IS sent.
    command_base_ = declare_parameter<bool>("command_base", true);

    // Safety clamps on the emitted velocity/acceleration command (per-DOF magnitude).
    max_base_vel_ = declare_parameter<double>("max_base_vel", 1.0);   // m/s, rad/s
    max_arm_vel_ = declare_parameter<double>("max_arm_vel", 2.0);     // rad/s
    max_base_acc_ = declare_parameter<double>("max_base_acc", 5.0);   // m/s^2, rad/s^2
    max_arm_acc_ = declare_parameter<double>("max_arm_acc", 10.0);    // rad/s^2
    vel_filter_alpha_ = declare_parameter<double>("vel_filter_alpha", 0.3);

    // --- Online-planning handshake (RT <-> OCS2) + front/back trajectories ---
    // RT requests planning (START_FRONT/START_BACK) over a dedicated XDDP RX port;
    // OCS2 streams setpoints and reports status (RUNNING/FAULT/DONE) in the desired
    // packet. The base CSVs are loaded in main(); OCS2 plans each from its row 0.
    csv_front_ = declare_parameter<std::string>("csv_front", "");
    csv_back_ = declare_parameter<std::string>("csv_back", "");
    cmd_port_ = declare_parameter<int>("xddp_cmd_port", 6);  // RT -> NRT MpcHandshake
    rt_task_cmd_ = declare_parameter<std::string>("rt_task_cmd", "tx_mpc_cmd_task");

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
    arm_qd_.assign(arm_joint_names_.size(), 0.0);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        base_pose_topic_, 10,
        std::bind(&HyummBridgeNode::poseCb, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        std::bind(&HyummBridgeNode::jointCb, this, std::placeholders::_1));

    desired_link_.configure("des", desired_port_, rt_task_desired_, send_desired_);
    desired_link_.setLogger(get_logger());
    desired_link_.poll();
    // Command RX (RT -> NRT). Always "enabled" so it reads when the RT task is up.
    cmd_link_.configure("mpc_cmd", cmd_port_, rt_task_cmd_, true);
    cmd_link_.setLogger(get_logger());
    cmd_link_.poll();
    watchdog_ = create_wall_timer(
        std::chrono::duration<double>(watchdog_period_),
        [this]() { desired_link_.poll(); cmd_link_.poll(); });

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

  // Overwrite the 18-D state [pos(9); vel(9)] with the latest measurements (leaves
  // entries untouched until data arrives, so the seed state is used meanwhile).
  void fillMeasuredState(ocs2::vector_t& state) const {
    if (have_base_) {
      state(hyumm_ocs2::state_idx::BASE_X) = base_x_;
      state(hyumm_ocs2::state_idx::BASE_Y) = base_y_;
      state(hyumm_ocs2::state_idx::BASE_PSI) = base_yaw_;        // continuous (unwrapped)
      state(hyumm_ocs2::VEL_OFFSET + 0) = base_vx_;              // base velocity (finite-diff)
      state(hyumm_ocs2::VEL_OFFSET + 1) = base_vy_;
      state(hyumm_ocs2::VEL_OFFSET + 2) = base_vyaw_;
    }
    if (have_arm_) {
      const size_t n = std::min<size_t>(arm_q_.size(), hyumm_ocs2::ARM_DIM);
      for (size_t i = 0; i < n; ++i) {
        state(hyumm_ocs2::state_idx::ARM_Q0 + i) = arm_q_[i];                                 // position
        state(hyumm_ocs2::VEL_OFFSET + hyumm_ocs2::BASE_STATE_DIM + i) = arm_qd_[i];          // velocity
      }
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

    // 2nd-order MPC: desired state xd = [pos(9); vel(9)], input ud = accel(9).
    // Send position, velocity AND acceleration so the RT side can quintic-
    // interpolate 100 Hz -> 1 kHz with smooth boundary conditions.
    packet::RobotInfo<MM_DOF> pkt{};   // {} zero-inits all sub-structs
    for (int i = 0; i < MM_DOF; ++i) {
      const bool isBase = (i < static_cast<int>(hyumm_ocs2::BASE_INPUT_DIM));
      double vel = xd(hyumm_ocs2::VEL_OFFSET + i);  // desired velocity (state)
      double acc = ud(i);                           // desired acceleration (input)
      const double vlim = isBase ? max_base_vel_ : max_arm_vel_;
      const double alim = isBase ? max_base_acc_ : max_arm_acc_;
      vel = std::max(-vlim, std::min(vlim, vel));   // safety clamps
      acc = std::max(-alim, std::min(alim, acc));
      if (isBase && !command_base_) { vel = 0.0; acc = 0.0; }
      vel *= rate_;                                 // phase-clock gate (1.0 unless stopping)
      acc *= rate_;

      double pos;
      if (use_int) { des_pos_int_[i] += vel * dt; pos = des_pos_int_[i]; }
      else { pos = xd(i); }                         // desired position (state)
      pkt.des.jointState.position[i] = pos;
      pkt.des.jointState.velocity[i] = vel;
      pkt.des.jointState.accel[i] = acc;
    }
    pkt.mpc.status = mpc_status_;
    pkt.mpc.fault = mpc_fault_;
    pkt.mpc.seq = ++mpc_seq_;
    desired_link_.tryWrite(pkt);
  }

  // Emit a hold packet (no motion, current status) -- used while IDLE/DONE/FAULT so
  // the RT side keeps seeing the handshake status (and a fresh seq) without motion.
  void sendHold(const ocs2::vector_t& measured) {
    if (!send_desired_) return;
    packet::RobotInfo<MM_DOF> pkt{};
    for (int i = 0; i < MM_DOF; ++i) {
      pkt.des.jointState.position[i] = i < measured.size() ? measured(i) : 0.0;
      pkt.des.jointState.velocity[i] = 0.0;
      pkt.des.jointState.accel[i] = 0.0;
    }
    pkt.mpc.status = mpc_status_;
    pkt.mpc.fault = mpc_fault_;
    pkt.mpc.seq = ++mpc_seq_;
    desired_link_.tryWrite(pkt);
  }

  // Read the latest RT command (drains to the most recent). Returns MPC_CMD_NONE if
  // nothing new arrived this cycle.
  uint8_t tryReadCmd() {
    packet::MpcHandshake hs{};
    uint8_t cmd = packet::MPC_CMD_NONE;
    while (cmd_link_.tryRead(hs)) cmd = hs.cmd;  // drain to latest
    return cmd;
  }
  void setMpcStatus(uint8_t status, uint16_t fault = packet::MPC_FAULT_NONE) {
    mpc_status_ = status;
    mpc_fault_ = fault;
  }
  const std::string& csvFront() const { return csv_front_; }
  const std::string& csvBack() const { return csv_back_; }

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

    // Base velocity by finite difference of the pose (the PoseStamped carries no
    // twist), lightly low-pass filtered. The 2nd-order MPC needs base velocity.
    const double tnow = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
    if (pose_t_init_) {
      const double ddt = tnow - prev_pose_t_;
      if (ddt > 1e-4) {
        const double a = vel_filter_alpha_;
        base_vx_ = a * (m->pose.position.x - base_x_) / ddt + (1.0 - a) * base_vx_;
        base_vy_ = a * (m->pose.position.y - base_y_) / ddt + (1.0 - a) * base_vy_;
        base_vyaw_ = a * (base_yaw_ - prev_base_yaw_) / ddt + (1.0 - a) * base_vyaw_;
      }
    }
    prev_pose_t_ = tnow;
    prev_base_yaw_ = base_yaw_;
    pose_t_init_ = true;
    base_x_ = m->pose.position.x;
    base_y_ = m->pose.position.y;
    have_base_ = true;
  }

  void jointCb(sensor_msgs::msg::JointState::ConstSharedPtr m) {
    size_t found = 0;
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      for (size_t j = 0; j < m->name.size(); ++j) {
        if (m->name[j] == arm_joint_names_[i] && j < m->position.size()) {
          arm_q_[i] = m->position[j];
          if (j < m->velocity.size()) arm_qd_[i] = m->velocity[j];  // 0 if velocity not published
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
  // handshake / front-back trajectories
  std::string csv_front_, csv_back_, rt_task_cmd_;
  int cmd_port_ = 6;
  uint8_t mpc_status_ = 0;   // packet::MpcStatus, sent in the desired packet
  uint16_t mpc_fault_ = 0;   // packet::MpcFault
  uint32_t mpc_seq_ = 0;     // setpoint sequence (liveness)
  double watchdog_period_ = 0.5;
  std::string desired_position_source_ = "state";
  bool command_base_ = true;
  double max_base_vel_ = 1.0, max_arm_vel_ = 2.0;
  double max_base_acc_ = 5.0, max_arm_acc_ = 10.0;  // accel safety clamps (m/s^2, rad/s^2)
  double vel_filter_alpha_ = 0.3;                   // base-velocity finite-diff low-pass

  // phase-clock collision-stop
  bool collision_stop_ = false;
  int collision_lookahead_ = 10;
  double rate_ramp_up_ = 0.02, rate_ramp_down_ = 0.20;
  double rate_ = 1.0;       // phase-clock rate in [0,1]
  bool int_init_ = false;   // desired-position integrator seeded?

  // measured state cache
  double base_x_ = 0.0, base_y_ = 0.0, base_yaw_ = 0.0;
  double base_vx_ = 0.0, base_vy_ = 0.0, base_vyaw_ = 0.0;  // finite-diff base velocity
  std::vector<double> arm_q_, arm_qd_;                       // arm position + velocity
  bool have_base_ = false, have_arm_ = false;
  bool yaw_init_ = false;
  double acc_yaw_ = 0.0, prev_yaw_ = 0.0;
  // base-velocity finite-diff state
  bool pose_t_init_ = false;
  double prev_pose_t_ = 0.0, prev_base_yaw_ = 0.0;
  double des_pos_int_[MM_DOF] = {0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  XddpLink desired_link_;
  XddpLink cmd_link_;   // RT -> NRT MpcHandshake command
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
  // Seed target is the 16-D packed reference [FK(seed) EE pose | seed joints] so the
  // EndEffectorConstraint (head 7) and PostureCost (tail 9) read consistent values.
  // A bare 9-D state here would feed the EE constraint a garbage 7-D "pose" (its
  // head) and yank the plan until the real CSV target arrives. See REF_DIM.
  const ocs2::TargetTrajectories initTarget(
      {obs.time}, {interface.makeReference(obs.state)}, {zeroInput});

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

  // Pre-load the front/back reference trajectories (each planned from its row 0).
  hyumm_ocs2::CsvTargetLoader loader;
  ocs2::TargetTrajectories front_ref, back_ref;
  bool have_front = false, have_back = false;
  try {
    if (!node->csvFront().empty()) { front_ref = loader.loadFromCsv(node->csvFront()); have_front = true; }
  } catch (const std::exception& e) { RCLCPP_ERROR(node->get_logger(), "front CSV: %s", e.what()); }
  try {
    if (!node->csvBack().empty()) { back_ref = loader.loadFromCsv(node->csvBack()); have_back = true; }
  } catch (const std::exception& e) { RCLCPP_ERROR(node->get_logger(), "back CSV: %s", e.what()); }
  RCLCPP_INFO(node->get_logger(), "trajectories: front=%s back=%s",
              have_front ? "loaded" : "none", have_back ? "loaded" : "none");

  // Handshake state machine.
  enum class St { IDLE, RUNNING, DONE, FAULT };
  St state = St::IDLE;
  node->setMpcStatus(packet::MPC_STAT_IDLE);
  double traj_end = 0.0;
  std::string seg = "-";

  const double dt = 1.0 / node->loopRateHz();
  rclcpp::Rate rate(node->loopRateHz());
  double t = 0.0;

  RCLCPP_INFO(node->get_logger(), "HYUMM bridge starting at %.0f Hz -- IDLE, waiting for RT START",
              node->loopRateHz());
  while (rclcpp::ok()) {
    mrt.spinMRT();   // services MPC policy callback AND our pose/joint_states/watchdog

    // --- handle RT command (start front/back, stop) ---
    const uint8_t cmd = node->tryReadCmd();
    if (cmd == packet::MPC_CMD_START_FRONT && state != St::RUNNING && have_front) {
      mrt.resetMpcNode(front_ref); t = 0.0; traj_end = front_ref.timeTrajectory.back();
      state = St::RUNNING; seg = "front"; node->setMpcStatus(packet::MPC_STAT_RUNNING);
      RCLCPP_INFO(node->get_logger(), "START_FRONT -> RUNNING (0..%.1fs)", traj_end);
    } else if (cmd == packet::MPC_CMD_START_BACK && state != St::RUNNING && have_back) {
      mrt.resetMpcNode(back_ref); t = 0.0; traj_end = back_ref.timeTrajectory.back();
      state = St::RUNNING; seg = "back"; node->setMpcStatus(packet::MPC_STAT_RUNNING);
      RCLCPP_INFO(node->get_logger(), "START_BACK -> RUNNING (0..%.1fs)", traj_end);
    } else if (cmd == packet::MPC_CMD_STOP) {
      state = St::IDLE; seg = "-"; node->setMpcStatus(packet::MPC_STAT_IDLE);
      RCLCPP_INFO(node->get_logger(), "STOP -> IDLE");
    }

    // --- measured observation -> MPC ---
    obs.time = t;
    node->fillMeasuredState(obs.state);
    mrt.setCurrentObservation(obs);
    mrt.updatePolicy();

    if (state == St::RUNNING) {
      if (mrt.initialPolicyReceived()) {
        ocs2::vector_t xd, ud;
        size_t mode = 0;
        mrt.evaluatePolicy(t, obs.state, xd, ud, mode);

        // collision -> FAULT (RT decides: hold / servo-off). Off unless enabled.
        bool hazard = false;
        if (monitor) {
          hazard = monitor->minDistance(obs.state) < monitor->getMinimumDistance();
          if (!hazard) {
            const auto& sol = mrt.getPolicy();
            const int N = std::min<int>(node->collisionLookahead(),
                                        static_cast<int>(sol.stateTrajectory_.size()));
            for (int k = 0; k < N; ++k)
              if (monitor->isInCollision(sol.stateTrajectory_[k])) { hazard = true; break; }
          }
        }
        if (hazard) {
          state = St::FAULT;
          node->setMpcStatus(packet::MPC_STAT_FAULT, packet::MPC_FAULT_COLLISION);
          RCLCPP_WARN(node->get_logger(), "COLLISION -> FAULT");
        } else {
          node->sendDesired(xd, ud, dt);    // status RUNNING written in the packet
          obs.input = ud;
          if (t >= traj_end) {
            state = St::DONE;
            node->setMpcStatus(packet::MPC_STAT_DONE);
            RCLCPP_INFO(node->get_logger(), "%s trajectory DONE", seg.c_str());
          } else {
            t += dt;
          }
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
              "[%s] t=%.1f/%.1f meas base[%.3f %.3f %.3f] des ddq0=%.2f",
              seg.c_str(), t, traj_end, obs.state(0), obs.state(1), obs.state(2), ud(hyumm_ocs2::BASE_INPUT_DIM));
        }
      } else {
        node->sendHold(obs.state);  // waiting for first policy
      }
    } else {
      // IDLE / DONE / FAULT: no motion, just report status (+ fresh seq) to RT.
      node->sendHold(obs.state);
    }

    rate.sleep();
  }

  mrt.shutdownNodes();
  rclcpp::shutdown();
  return 0;
}
