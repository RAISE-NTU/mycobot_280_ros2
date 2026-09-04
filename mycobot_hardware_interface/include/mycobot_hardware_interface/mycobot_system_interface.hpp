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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mycobot_hardware_interface/mycobot_serial.hpp"

namespace mycobot_hardware_interface
{

/// @brief ros2_control SystemInterface plugin for the myCobot 280 M5.
///
/// Communicates with the arm over USB serial using the myCobot binary protocol.
/// Provides position-only state and command interfaces for 6 joints.
///
/// Hardware parameters (from XACRO <param> tags):
///   - serial_port: e.g. "/dev/ttyUSB0"
///   - baud_rate:   e.g. "115200"
class MyCobotSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCobotSystemInterface)

  // --- Lifecycle ---

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Called by controller_manager when read()/write() returns ERROR.
  ///
  /// This is NOT on_deactivate() — ros2_control routes the error path here, so the
  /// servo release and port close must be duplicated. Without this override an
  /// ERROR return leaves the servos energised and the port open.
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Interface export ---

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- Control loop ---

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  std::unique_ptr<MyCobotSerial> serial_;
  std::string serial_port_;
  int baud_rate_{115200};

  // Joint buffers, sized to every joint declared in the <ros2_control> block.
  // Indices [0, NUM_JOINTS) are the arm joints and are the only ones backed by real
  // hardware; any joint beyond that is passive (see export_state_interfaces).
  std::vector<double> hw_position_states_;    // radians (read from hardware)
  std::vector<double> hw_position_commands_;  // radians (written to hardware)
  std::vector<double> hw_velocity_states_;    // rad/s — passive joints only, always 0
  std::vector<double> last_sent_angles_;      // degrees (last sent target)

  // Write pacing — wall-clock time of the last send_angles() actually issued.
  std::chrono::steady_clock::time_point last_write_time_{};

  // Error tracking
  int consecutive_read_errors_{0};
  bool clamp_warned_{false};  // edge-trigger for the out-of-range clamp warning

  // Gripper (adaptive gripper on the flange). Only populated when use_gripper:=true
  // adds the 'gripper_controller' joint to the <ros2_control> block.
  //   gripper_cmd_index_    : buffer index of gripper_controller, or -1 if absent.
  //   last_sent_gripper_value_ : last 0-100 value written, so we only resend on change.
  int gripper_cmd_index_{-1};
  int last_sent_gripper_value_{-1};

  static constexpr size_t NUM_JOINTS = 6;

  // Gripper joint <-> firmware value mapping. The URDF gripper_controller joint runs
  // GRIPPER_OPEN_RAD (physically open) to GRIPPER_CLOSED_RAD (closed); the firmware
  // takes 0 (closed) .. 100 (open). value = 100*(pos-closed)/(open-closed), clamped.
  static constexpr double GRIPPER_OPEN_RAD = 0.0;
  static constexpr double GRIPPER_CLOSED_RAD = -0.5;
  static constexpr int GRIPPER_SPEED = 50;   // 1-100
  static constexpr int GRIPPER_TYPE = 1;     // 1 = adaptive gripper

  // percent of max speed (1–100). Lowered from 50 to 25: the myCobot servos trip to a
  // blue-LED torque/over-current fault when driven hard into deep or loaded poses, and
  // the trip threshold rises as speed drops. 25% gives J4 the headroom to reach box-grasp
  // poses (and hold the payload) without faulting. Raise again if moves feel too slow AND
  // the deep reaches still complete cleanly.
  static constexpr int DEFAULT_SPEED = 25;

  /// Minimum wall-clock gap between two send_angles() commands, in milliseconds.
  ///
  /// The controller runs write() every 100 ms (10 Hz) and the trajectory setpoint
  /// changes on every cycle, so the naive interface fired a SEND_ANGLES each cycle.
  /// The myCobot firmware does its OWN point-to-point interpolation to whatever target
  /// it last received, so a fresh target every 100 ms just restarts the move before it
  /// finishes AND floods the ESP32 RX buffer while it is busy driving servos — the
  /// buffer overflows, the final goal frame is dropped, and the arm stops mid-path
  /// while ros2_control (feedback frozen by the read timeouts) still reports SUCCEEDED.
  /// Pacing sends to ~4 Hz lets the firmware keep up and guarantees the settled goal
  /// (which is held after the trajectory ends) is delivered cleanly. Tune against the
  /// scratchpad `soak --mode wiggle` tool. Still coarse; the real streaming fix would
  /// send trajectory endpoints, but this makes point-to-point moves complete reliably.
  static constexpr int MIN_WRITE_INTERVAL_MS = 250;
  /// Consecutive failed reads before escalating to ERROR (servo release + port close).
  ///
  /// At the 15 Hz controller rate this is ~0.67 s of stale feedback. The previous
  /// value of 3 (~0.2 s) proved too twitchy: a 176 s soak recorded zero steady-state
  /// read failures and a worst-case latency of 16.0 ms, so any short burst is far more
  /// likely to be CPU starvation or transient bus contention than a real disconnect.
  /// A genuine unplug still trips this in well under a second.
  static constexpr int MAX_CONSECUTIVE_ERRORS = 10;

  /// Wall-clock budget for the ESP32 boot/transponder handshake during on_activate.
  static constexpr int HANDSHAKE_TIMEOUT_S = 45;
};

}  // namespace mycobot_hardware_interface
