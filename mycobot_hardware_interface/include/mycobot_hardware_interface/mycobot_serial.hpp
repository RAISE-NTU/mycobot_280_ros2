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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mycobot_hardware_interface
{

/// @brief Low-level serial protocol layer for the myCobot 280 M5.
///
/// Implements the binary serial protocol documented in the pymycobot source code.
/// This class has no ROS2 dependencies, making it independently testable.
///
/// Frame format (verified against pymycobot/common.py, 2026-02-25):
///   [0xFE][0xFE][LEN][CMD_ID][DATA...][0xFA]
///   LEN = 1 (cmd_id) + len(data) + 1 (footer) = len(data) + 2
///
/// Angle encoding: degrees × 100, as signed 16-bit big-endian integer.
class MyCobotSerial
{
public:
  MyCobotSerial();
  ~MyCobotSerial();

  // Non-copyable, non-movable (owns serial port resource)
  MyCobotSerial(const MyCobotSerial &) = delete;
  MyCobotSerial & operator=(const MyCobotSerial &) = delete;

  /// Open the serial port. Throws std::runtime_error on failure.
  void open(const std::string & port, int baud_rate);

  /// Close the serial port.
  void close();

  /// Returns true if the serial port is currently open.
  bool is_open() const;

  /// Read current joint angles (6 values, in degrees).
  /// Returns false on timeout or invalid response.
  bool get_angles(std::vector<double> & angles_deg);

  /// Send target joint angles (6 values, in degrees) at given speed (1–100).
  /// Returns false on serial write failure.
  bool send_angles(const std::vector<double> & angles_deg, int speed);

  /// Check if myCobot power is on (command 0x12).
  bool is_power_on();

  /// Release all servo motors (command 0x13).
  bool release_all_servos();

  /// Re-enable all servo motors (sends FOCUS_SERVO 0x57 with servo_id=0).
  bool focus_all_servos();

  /// Drive the adaptive gripper to an opening value (0 = closed, 100 = open).
  /// speed is 1–100; gripper_type selects the fitted gripper (1 = adaptive).
  /// Sends SET_GRIPPER_VALUE 0x67 with payload [value, speed, gripper_type].
  /// Fire-and-forget like send_angles(): does not wait for an ack.
  bool set_gripper_value(int value, int speed, int gripper_type = 1);

  /// Read the gripper's current opening value (0–100) into `value`.
  /// Sends GET_GRIPPER_VALUE 0x65; returns false on timeout/invalid reply.
  bool get_gripper_value(int & value, int gripper_type = 1);

  // --- Protocol constants (public for unit testing) ---

  static constexpr uint8_t HEADER = 0xFE;
  static constexpr uint8_t FOOTER = 0xFA;

  // Command IDs (verified against pymycobot/common.py, 2026-02-25)
  static constexpr uint8_t CMD_IS_POWER_ON        = 0x12;
  static constexpr uint8_t CMD_GET_ANGLES         = 0x20;
  static constexpr uint8_t CMD_SEND_ANGLES         = 0x22;
  static constexpr uint8_t CMD_RELEASE_ALL_SERVOS  = 0x13;
  static constexpr uint8_t CMD_FOCUS_SERVO         = 0x57;
  static constexpr uint8_t CMD_GET_GRIPPER_VALUE   = 0x65;
  static constexpr uint8_t CMD_SET_GRIPPER_VALUE   = 0x67;

  static constexpr size_t NUM_JOINTS = 6;

  /// Deadline for a query response, in milliseconds.
  ///
  /// A GET_ANGLES reply is 17 bytes (~1.5 ms on the wire at 115200 baud), so this is
  /// almost entirely firmware turnaround budget. Measured on a myCobot 280 M5
  /// (2026-07-21): while the arm is STATIONARY, latency is 13.1 ms median / 14.1 ms
  /// max. While the firmware is DRIVING SERVOS the distribution becomes bimodal —
  /// ~95% of samples stay near 14 ms, but a tail lands at 58-60 ms. A 50 ms deadline
  /// truncated that tail, turning healthy-but-slow replies into read failures and
  /// tripping the error escalation mid-trajectory.
  ///
  /// 80 ms clears the 59.6 ms observed maximum with ~35% headroom. read_response()
  /// only returns early on a complete frame, so this is also the worst-case stall of
  /// the read() cycle: it must fit inside the controller period alongside the write.
  /// At the 10 Hz update_rate (100 ms period) it does; at 15 Hz it would not.
  static constexpr int QUERY_TIMEOUT_MS = 80;

  /// Upper bound on the persistent RX buffer, in bytes.
  /// Only reached if the firmware goes silent or emits frames we never consume; the
  /// oldest bytes are dropped so a stuck link cannot grow the buffer without limit.
  static constexpr size_t MAX_RX_BUFFER = 512;

  // --- Static helpers (public for unit testing) ---

  /// Encode a double angle (degrees) to int16 big-endian (degrees × 100).
  static void encode_angle(double degrees, uint8_t & high, uint8_t & low);

  /// Decode int16 big-endian (degrees × 100) to double angle (degrees).
  static double decode_angle(uint8_t high, uint8_t low);

  /// Build a complete command frame: [0xFE][0xFE][LEN][cmd_id][data...][0xFA].
  static std::vector<uint8_t> build_frame(
    uint8_t command_id, const std::vector<uint8_t> & data = {});

  /// Parse a response frame. Returns true if valid, populates cmd_id and payload.
  ///
  /// The firmware may emit unsolicited or lagging frames (e.g. a POWER_ON 0x10
  /// reply) in the same burst as the reply we asked for, so the first frame in the
  /// buffer is not necessarily the right one. Pass `expected_cmd` to keep scanning
  /// until a frame with that command id is found; -1 (default) returns the first
  /// well-formed frame regardless of command id.
  static bool parse_frame(
    const std::vector<uint8_t> & buffer,
    uint8_t & cmd_id,
    std::vector<uint8_t> & payload,
    int expected_cmd = -1);

  /// Find the LAST complete frame matching `expected_cmd` in `buffer`.
  ///
  /// Unlike parse_frame(), which stops at the first match, this returns the freshest
  /// matching frame and reports where it ends via `frame_end` (one past its final
  /// byte) so the caller can consume everything up to and including it.
  ///
  /// Taking the last match matters: when the firmware falls behind, the buffer can
  /// hold a stale reply followed by a current one. Returning the first would feed the
  /// controller progressively older joint angles.
  static bool find_last_frame(
    const std::vector<uint8_t> & buffer,
    int expected_cmd,
    uint8_t & cmd_id,
    std::vector<uint8_t> & payload,
    size_t & frame_end);

private:
  /// Send a command frame over the serial port.
  bool send_command(uint8_t command_id, const std::vector<uint8_t> & data = {});

  /// Read a response frame with timeout. Populates cmd_id and payload on success.
  /// `expected_cmd` is forwarded to parse_frame() so that interleaved frames from
  /// the firmware do not mask the reply we are waiting for.
  bool read_response(uint8_t & cmd_id, std::vector<uint8_t> & payload,
                     int timeout_ms = 200, int expected_cmd = -1);

  struct Impl;  // PIMPL for libserial types
  std::unique_ptr<Impl> impl_;
};

}  // namespace mycobot_hardware_interface
