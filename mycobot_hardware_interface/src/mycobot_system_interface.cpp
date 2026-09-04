// Copyright 2026 Nottingham Trent University — COMP40761 Cognitive Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mycobot_hardware_interface/mycobot_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mycobot_hardware_interface
{

namespace
{
// --- Firmware <-> URDF joint calibration (servo zeros RECALIBRATED 2026-07-30) ---
//
// Per-joint affine map:
//
//     urdf_rad[i] = JOINT_SIGN[i] * (firmware_deg[i] - FW_HOME_DEG[i]) * pi/180
//
// FW_HOME_DEG = firmware reading with the arm at the URDF home (straight vertical
// column). After the 2026-07-30 servo zero-point recalibration (myStudio Calibration:
// each joint aligned to its groove/zero mark, then Transponder re-flashed), the arm's
// firmware zero IS the vertical home: commanding send_angles([0,0,0,0,0,0]) drives it
// to the vertical column and it reads back ~[0,0,0,0,0,0]. So every FW_HOME is 0.
//
// This SUPERSEDES the pre-recalibration table {90.70, 7.99, -28.47, -123.83, -55.11,
// 0.70}. Those large offsets existed because the OLD servo zeros were badly off (the
// firmware zero was a bent pose) and J1/J5 carried a +/-90 yaw fudge. Re-zeroing the
// servos removes all of that AND re-centers each joint's +/-spec range on home, which
// is what restores J1's full +/-165 reach (the whole reason for recalibrating).
//
// Any residual mounting-yaw mismatch (real arm facing a rotated compass direction from
// the RViz twin at home) must be corrected in the BASE FRAME (urdf), NOT by re-adding a
// FW_HOME offset -- an offset here would re-shift the reachable window and eat back into
// the range we just recovered.
//
// SIGN + any yaw offset are VERIFIED in RViz with the method that actually works: view
// both RViz and the real arm straight down (+Z), move ONE joint via the Joints-tab
// slider, and check rotation direction from that shared top-down viewpoint. Side/
// eye-level compares are unreliable. Signs start all +1 (firmware and urdf turned the
// same way pre-recal); confirm per-joint after rebuild.
constexpr double FW_HOME_DEG[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr double JOINT_SIGN[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// Per-joint firmware drive limits (degrees). write() clamps every target to these so we
// NEVER command a joint past its spec — commanding past spec is exactly what trips the
// blue overload (a trip stalls the arm and latches the servo off until a relaunch).
//
// These are the OFFICIAL myCobot 280 limits taken from pymycobot (robot_info.py,
// "MyCobot280": angles_min/angles_max = J1 ±168, J2 ±140, J3 ±150, J4 ±150,
// J5 -155/+160, J6 ±180). pymycobot itself REFUSES to send a target outside them; our
// interface had no such guard, so commanding J4 past -150 (we sent -152.8 / -163) drove
// it out of spec and flipped the Atom to blue. CONFIRMED on hardware 2026-07-27 with the
// pymycobot scratchpad test: J4 drives cleanly to -147.6 at 25% speed with NO blue, then
// pymycobot blocks -151.9 as out of range. So -150 (= urdf ~-26) is a HARD spec floor, not
// a torque/speed artefact, and cannot be "opened up". A pose that needs a joint past its
// limit must be reached via wrist redundancy (J5/J6), never by loosening this.
//
// Symmetric clamp uses each joint's TIGHTER spec bound so it stays in spec on both sides
// (J2 was 165 -> 140, J5 was 160 -> 155; these were looser than spec and could have
// tripped). J1 (160<168) and J6 (174<180, and URDF caps J6 anyway) stay conservative.
constexpr double FW_LIMIT_DEG[6] = {160.0, 140.0, 150.0, 150.0, 155.0, 174.0};
}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle: on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MyCobotSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call base class initialisation first
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Read hardware parameters from XACRO <param> tags ---
  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Missing required hardware parameter 'serial_port'.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Hardware parameter 'baud_rate' not specified. Using default: %d.", baud_rate_);
  }

  // --- Validate joint configuration ---
  if (info_.joints.size() < NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Expected at least %zu joints, got %zu.", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Only the first NUM_JOINTS joints are driven over serial, and they must expose a
  // position command + position state interface. Joints declared beyond that (the
  // adaptive gripper and its five mimic joints) are PASSIVE: they are claimed so the
  // MoveIt planning scene sees a complete robot state, but they are never read from
  // or written to the hardware. Their interfaces are accepted as declared.
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    const auto & joint = info_.joints[i];

    // Each arm joint must have exactly one position command interface
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Arm joint '%s' must have exactly one 'position' command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Each arm joint must have exactly one position state interface
    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Arm joint '%s' must have exactly one 'position' state interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Joints beyond the arm belong to the gripper. The 'gripper_controller' joint carries
  // a position command interface and IS actuated (mapped to SET_GRIPPER_VALUE); its five
  // mimic joints declare no command interface and stay passive — robot_state_publisher
  // derives them from gripper_controller via the URDF <mimic> tags.
  for (size_t i = NUM_JOINTS; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    bool has_pos_cmd = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) { has_pos_cmd = true; break; }
    }
    if (has_pos_cmd) {
      gripper_cmd_index_ = static_cast<int>(i);
      RCLCPP_INFO(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Joint '%s' (index %zu) is the ACTIVE gripper: driven via SET_GRIPPER_VALUE.",
        joint.name.c_str(), i);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Joint '%s' is passive: claimed for state completeness, never actuated.",
        joint.name.c_str());
    }
  }

  // --- Allocate buffers for all exported joints ---
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_velocity_states_.resize(info_.joints.size(), 0.0);

  // --- Construct serial object (port is NOT opened yet) ---
  serial_ = std::make_unique<MyCobotSerial>();

  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Initialised for port '%s' at %d baud with %zu joints.",
    serial_port_.c_str(), baud_rate_, NUM_JOINTS);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MyCobotSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Activating: opening serial port '%s'...", serial_port_.c_str());

  // Open the serial port
  try {
    serial_->open(serial_port_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Failed to open serial port '%s': %s", serial_port_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify connection by reading current joint angles (retry loop to allow ESP32 boot/handshake)
  std::vector<double> angles_deg;
  bool read_success = false;
  
  // Send initial handshake ping once to wake up transponder firmware thread
  serial_->is_power_on();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Bound the handshake by wall-clock time, not attempt count: get_angles() uses a
  // short per-query timeout tuned for the control loop, so an attempt budget would
  // silently shrink this window if that timeout is ever retuned.
  const auto handshake_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(HANDSHAKE_TIMEOUT_S);
  int attempt = 0;
  while (std::chrono::steady_clock::now() < handshake_deadline) {
    if (serial_->get_angles(angles_deg)) {
      read_success = true;
      break;
    }
    RCLCPP_WARN(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Waiting for myCobot to boot and handshake (attempt %d, %d s budget)...",
      ++attempt, HANDSHAKE_TIMEOUT_S);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (!read_success) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Failed to read initial joint angles. Is the myCobot powered on and in Transponder mode?");
    serial_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Re-engage the servos before seeding the command buffers.
  //
  // on_deactivate()/on_error() send RELEASE_ALL_SERVOS, so after any previous shutdown
  // the arm is limp and will have sagged under gravity. Without this the interface
  // would activate against a drooping arm and hold it there. Focus first, then re-read,
  // so the buffers are seeded from the position the servos actually latched at rather
  // than the pre-focus sagged reading.
  if (!serial_->focus_all_servos()) {
    RCLCPP_WARN(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Failed to send FOCUS_SERVO; servos may be limp and the arm unsupported.");
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::vector<double> settled_deg;
    if (serial_->get_angles(settled_deg)) {
      angles_deg = settled_deg;
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Could not re-read angles after focusing servos; seeding from pre-focus values.");
    }
  }

  // Populate state and command buffers with current positions
  // (so the first write() does not cause a jump). Apply the same firmware->URDF
  // calibration as read(), or the seeded command would be raw firmware radians and
  // write() would invert the calibration onto it, jerking the arm on cycle one.
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_position_states_[i]   =
      JOINT_SIGN[i] * (angles_deg[i] - FW_HOME_DEG[i]) * M_PI / 180.0;
    hw_position_commands_[i] = hw_position_states_[i];
  }
  for (size_t i = NUM_JOINTS; i < info_.joints.size(); ++i) {
    hw_position_states_[i]   = 0.0;
    hw_position_commands_[i] = 0.0;
  }

  // Seed the gripper from its actual opening so the first write() does not snap it.
  last_sent_gripper_value_ = -1;
  if (gripper_cmd_index_ >= 0) {
    int gval = 0;
    double seed_rad = GRIPPER_OPEN_RAD;
    if (serial_->get_gripper_value(gval, GRIPPER_TYPE)) {
      gval = std::max(0, std::min(100, gval));
      seed_rad = GRIPPER_CLOSED_RAD +
        (static_cast<double>(gval) / 100.0) * (GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Could not read initial gripper value; seeding gripper command to open.");
    }
    hw_position_states_[gripper_cmd_index_]   = seed_rad;
    hw_position_commands_[gripper_cmd_index_] = seed_rad;
  }

  consecutive_read_errors_ = 0;
  last_sent_angles_.clear();

  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Calibration ACTIVE  FW_HOME_DEG=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]  "
    "SIGN=[%+.0f, %+.0f, %+.0f, %+.0f, %+.0f, %+.0f]",
    FW_HOME_DEG[0], FW_HOME_DEG[1], FW_HOME_DEG[2],
    FW_HOME_DEG[3], FW_HOME_DEG[4], FW_HOME_DEG[5],
    JOINT_SIGN[0], JOINT_SIGN[1], JOINT_SIGN[2],
    JOINT_SIGN[3], JOINT_SIGN[4], JOINT_SIGN[5]);

  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Drive config: DEFAULT_SPEED=%d%%  FW_LIMIT_DEG=[%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]  "
    "(J4 negative clamp = firmware %.0f deg = urdf %.1f deg)",
    DEFAULT_SPEED, FW_LIMIT_DEG[0], FW_LIMIT_DEG[1], FW_LIMIT_DEG[2],
    FW_LIMIT_DEG[3], FW_LIMIT_DEG[4], FW_LIMIT_DEG[5],
    -FW_LIMIT_DEG[3], (-FW_LIMIT_DEG[3] - FW_HOME_DEG[3]));

  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Activated. Initial joint angles (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    angles_deg[0], angles_deg[1], angles_deg[2],
    angles_deg[3], angles_deg[4], angles_deg[5]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MyCobotSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Deactivating: releasing servos and closing serial port...");

  // Release servos so the arm goes limp (safe shutdown)
  if (serial_ && serial_->is_open()) {
    if (!serial_->release_all_servos()) {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Failed to send RELEASE_ALL_SERVOS command.");
    }
    serial_->close();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_error
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MyCobotSystemInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("MyCobotSystemInterface"),
    "Entered error state: releasing servos and closing serial port...");

  // Best-effort: the serial link is by definition unhealthy here, so a failed
  // release is expected and must not prevent the port from being closed.
  if (serial_ && serial_->is_open()) {
    if (!serial_->release_all_servos()) {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Could not send RELEASE_ALL_SERVOS while entering error state; "
        "servos may remain energised.");
    }
    serial_->close();
  }

  consecutive_read_errors_ = 0;
  last_sent_angles_.clear();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
MyCobotSystemInterface::export_state_interfaces()
{
  // Export exactly what the <ros2_control> block declares. The gripper joints ask for
  // a velocity state interface as well as position; failing to export an interface the
  // URDF declares makes resource_manager abort at startup.
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &hw_position_states_[i]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &hw_velocity_states_[i]);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("MyCobotSystemInterface"),
          "Joint '%s' declares unsupported state interface '%s'; not exported.",
          info_.joints[i].name.c_str(), si.name.c_str());
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotSystemInterface::export_command_interfaces()
{
  // Mimic joints declare no command interface at all, so iterate what is declared
  // rather than assuming one command interface per joint.
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &hw_position_commands_[i]);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("MyCobotSystemInterface"),
          "Joint '%s' declares unsupported command interface '%s'; not exported.",
          info_.joints[i].name.c_str(), ci.name.c_str());
      }
    }
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Control loop: read
// ---------------------------------------------------------------------------
hardware_interface::return_type MyCobotSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> angles_deg;
  if (serial_->get_angles(angles_deg)) {
    // Successful read — map firmware degrees to URDF radians (see calibration above)
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      hw_position_states_[i] =
        JOINT_SIGN[i] * (angles_deg[i] - FW_HOME_DEG[i]) * M_PI / 180.0;
    }
    consecutive_read_errors_ = 0;
  } else {
    // Failed read — HOLD the last known state. A read timeout must NEVER tear the
    // robot down.
    //
    // This firmware services servo motion before it answers GET_ANGLES, so while the
    // arm is moving replies stall, and for ~1 s at the end of a trajectory (while it
    // finishes the final move) they stop entirely. Returning ERROR here drives
    // ros2_control into on_error() -> RELEASE_ALL_SERVOS, dropping a focused arm limp
    // on a transient stall — which is exactly what happened right after a SUCCEEDED
    // trajectory. Holding the last commanded pose with servos still focused is the
    // safe response; a stale reading is harmless, a limp arm is not. We only log,
    // throttled, so a genuinely dead USB link stays visible without spamming every
    // 100 ms control cycle.
    ++consecutive_read_errors_;
    if (consecutive_read_errors_ == 1 ||
        consecutive_read_errors_ % MAX_CONSECUTIVE_ERRORS == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Serial read timed out %d cycle(s) in a row; holding last known state "
        "(servos stay focused). Sustained timeouts mean the firmware is busy moving "
        "or the USB link is down.",
        consecutive_read_errors_);
    }
  }

  // Gripper runs open-loop: mirror its command into its state so the
  // GripperActionController sees the commanded opening reached. Polling
  // get_gripper_value every cycle would contend with GET_ANGLES on the same link and
  // its 17-96 physical span never reaches the 0/100 endpoints the action waits on.
  if (gripper_cmd_index_ >= 0) {
    hw_position_states_[gripper_cmd_index_] = hw_position_commands_[gripper_cmd_index_];
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Control loop: write
// ---------------------------------------------------------------------------
hardware_interface::return_type MyCobotSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert URDF radians back to firmware degrees (inverse of read() mapping), then
  // clamp to the servo range so an unreachable target cannot fault the arm.
  std::vector<double> angles_deg(NUM_JOINTS);
  bool clamped = false;
  int clamped_joint = 0;
  double clamped_target = 0.0;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    const double urdf_deg = hw_position_commands_[i] * 180.0 / M_PI;
    double fw = FW_HOME_DEG[i] + JOINT_SIGN[i] * urdf_deg;
    if (fw > FW_LIMIT_DEG[i]) {
      clamped = true; clamped_joint = static_cast<int>(i); clamped_target = fw;
      fw = FW_LIMIT_DEG[i];
    } else if (fw < -FW_LIMIT_DEG[i]) {
      clamped = true; clamped_joint = static_cast<int>(i); clamped_target = fw;
      fw = -FW_LIMIT_DEG[i];
    }
    angles_deg[i] = fw;
  }

  // Edge-triggered log: warn once when a target first goes out of range, not every
  // cycle. If this fires when the arm "stops midway", the commanded pose is past the
  // servo's +/-spec limit. With FW_HOME=0 (recalibrated) the clamp window is centred on
  // home, so this should only trip at genuine range extremes; a pose that needs more
  // must be reached via wrist redundancy (J5/J6), never by loosening FW_LIMIT_DEG.
  if (clamped && !clamp_warned_) {
    RCLCPP_WARN(
      rclcpp::get_logger("MyCobotSystemInterface"),
      "Joint %d target %.1f deg exceeds the myCobot servo limit (+/-%.0f deg firmware) "
      "and was CLAMPED. The real arm cannot reach the commanded pose; it will stop at "
      "the limit while RViz shows the full target.",
      clamped_joint + 1, clamped_target, FW_LIMIT_DEG[clamped_joint]);
    clamp_warned_ = true;
  } else if (!clamped) {
    clamp_warned_ = false;
  }

  bool needs_update = false;
  if (last_sent_angles_.size() != NUM_JOINTS) {
    needs_update = true;
  } else {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(angles_deg[i] - last_sent_angles_[i]) > 0.02) {
        needs_update = true;
        break;
      }
    }
  }

  // Pace the writes: never issue SEND_ANGLES more often than MIN_WRITE_INTERVAL_MS,
  // even though the controller calls write() every 10 Hz cycle with a fresh setpoint.
  // Flooding the firmware every cycle overflows its RX buffer mid-trajectory and drops
  // the final goal frame (arm stops mid-path). The trajectory's final target is HELD
  // after execution, so it still differs from last_sent_angles_ and gets delivered on
  // the next paced cycle — the move completes.
  const auto now = std::chrono::steady_clock::now();
  const bool interval_elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_write_time_)
      .count() >= MIN_WRITE_INTERVAL_MS;

  if (needs_update && interval_elapsed) {
    if (!serial_->send_angles(angles_deg, DEFAULT_SPEED)) {
      RCLCPP_WARN(
        rclcpp::get_logger("MyCobotSystemInterface"),
        "Failed to send joint angles to myCobot.");
    } else {
      last_sent_angles_ = angles_deg;
      last_write_time_ = now;
    }
  }

  // Gripper: map commanded radians to the firmware's 0 (closed) .. 100 (open) opening
  // and send only when the target changes. Grasp open/close is infrequent, so no time
  // pacing is needed — the value-change guard alone prevents flooding the link.
  if (gripper_cmd_index_ >= 0) {
    const double pos = hw_position_commands_[gripper_cmd_index_];
    const double frac =
      (pos - GRIPPER_CLOSED_RAD) / (GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD);
    int value = static_cast<int>(std::lround(frac * 100.0));
    value = std::max(0, std::min(100, value));
    if (value != last_sent_gripper_value_) {
      if (serial_->set_gripper_value(value, GRIPPER_SPEED, GRIPPER_TYPE)) {
        RCLCPP_INFO(
          rclcpp::get_logger("MyCobotSystemInterface"),
          "Gripper command: value %d (pos %.3f rad) sent over serial "
          "[read_errors=%d].",
          value, pos, consecutive_read_errors_);
        last_sent_gripper_value_ = value;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("MyCobotSystemInterface"),
          "Failed to send gripper value %d.", value);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mycobot_hardware_interface

// ---------------------------------------------------------------------------
// pluginlib class loader registration
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mycobot_hardware_interface::MyCobotSystemInterface,
  hardware_interface::SystemInterface)
