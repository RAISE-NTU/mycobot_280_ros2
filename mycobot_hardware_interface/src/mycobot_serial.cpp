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

#include "mycobot_hardware_interface/mycobot_serial.hpp"

#include <libserial/SerialPort.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <thread>

namespace mycobot_hardware_interface
{

// ---------------------------------------------------------------------------
// PIMPL — keeps libserial headers out of the public interface
// ---------------------------------------------------------------------------
struct MyCobotSerial::Impl
{
  LibSerial::SerialPort port;
  bool port_open{false};

  /// Bytes carried over between calls. The firmware can lag by more than one control
  /// cycle; discarding the buffer each cycle would throw away the reply we are waiting
  /// for and make it impossible to ever resynchronise.
  std::vector<uint8_t> rx;
};

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------
MyCobotSerial::MyCobotSerial()
: impl_(std::make_unique<Impl>())
{
}

MyCobotSerial::~MyCobotSerial()
{
  if (impl_ && impl_->port_open) {
    try {
      close();
    } catch (...) {
      // Suppress exceptions in destructor
    }
  }
}

// ---------------------------------------------------------------------------
// Port management
// ---------------------------------------------------------------------------
void MyCobotSerial::open(const std::string & port, int baud_rate)
{
  if (impl_->port_open) {
    throw std::runtime_error("Serial port is already open");
  }

  impl_->port.Open(port);

  // Map integer baud rate to libserial enum
  LibSerial::BaudRate baud;
  switch (baud_rate) {
    case 9600:   baud = LibSerial::BaudRate::BAUD_9600; break;
    case 19200:  baud = LibSerial::BaudRate::BAUD_19200; break;
    case 38400:  baud = LibSerial::BaudRate::BAUD_38400; break;
    case 57600:  baud = LibSerial::BaudRate::BAUD_57600; break;
    case 115200: baud = LibSerial::BaudRate::BAUD_115200; break;
    default:
      impl_->port.Close();
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate));
  }

  impl_->port.SetBaudRate(baud);
  impl_->port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  impl_->port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  impl_->port.SetParity(LibSerial::Parity::PARITY_NONE);
  impl_->port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

  // Explicitly clear DTR and RTS lines to release ESP32 microcontroller from reset mode
  try {
    impl_->port.SetDTR(false);
    impl_->port.SetRTS(false);
  } catch (...) {}

  impl_->port_open = true;

  // Brief delay to allow port stabilization
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Flush any startup/reset bootloader message garbage from the buffer
  try {
    impl_->port.FlushInputBuffer();
    impl_->port.FlushOutputBuffer();
  } catch (...) {
    // Ignore flush errors
  }
  impl_->rx.clear();
}

void MyCobotSerial::close()
{
  if (impl_->port_open) {
    impl_->port.Close();
    impl_->port_open = false;
  }
}

bool MyCobotSerial::is_open() const
{
  return impl_->port_open;
}

// ---------------------------------------------------------------------------
// High-level commands
// ---------------------------------------------------------------------------
bool MyCobotSerial::get_angles(std::vector<double> & angles_deg)
{
  if (!impl_->port_open) {
    return false;
  }

  // NOTE: deliberately no FlushInputBuffer() here.
  //
  // Flushing before each request was the cause of a hard failure mode: once the
  // firmware fell more than one control cycle behind (which it does during motion),
  // the flush discarded the very reply being waited for. The next cycle flushed it
  // again, so the link could never resynchronise and produced unbroken runs of
  // timeouts until the error escalation tore the robot down. Instead, bytes are
  // accumulated across cycles and a late reply is accepted rather than binned.
  if (!send_command(CMD_GET_ANGLES)) {
    return false;
  }

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  size_t frame_end = 0;

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(QUERY_TIMEOUT_MS);

  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      uint8_t byte;
      impl_->port.ReadByte(byte, 5);
      impl_->rx.push_back(byte);
    } catch (...) {
      // No byte within the per-byte timeout; keep polling until the deadline.
    }

    if (find_last_frame(impl_->rx, CMD_GET_ANGLES, cmd_id, payload, frame_end)) {
      found = true;
      break;
    }
  }

  if (found) {
    // Consume through the frame we used, discarding anything older (stale replies
    // and SEND_ANGLES acks alike).
    impl_->rx.erase(impl_->rx.begin(), impl_->rx.begin() + frame_end);
  } else if (impl_->rx.size() > MAX_RX_BUFFER) {
    // Nothing parseable and the buffer is growing: drop the oldest half so a silent
    // or babbling link cannot consume memory without bound.
    impl_->rx.erase(impl_->rx.begin(), impl_->rx.begin() + impl_->rx.size() / 2);
  }

  if (!found || payload.size() != NUM_JOINTS * 2) {
    return false;
  }

  angles_deg.resize(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    angles_deg[i] = decode_angle(payload[i * 2], payload[i * 2 + 1]);
  }
  return true;
}

bool MyCobotSerial::send_angles(const std::vector<double> & angles_deg, int speed)
{
  if (angles_deg.size() != NUM_JOINTS) {
    return false;
  }

  // Clamp speed to 1–100
  int clamped_speed = std::max(1, std::min(100, speed));

  // Build data: 12 bytes (6 × int16 big-endian) + 1 byte speed
  std::vector<uint8_t> data;
  data.reserve(NUM_JOINTS * 2 + 1);

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint8_t high, low;
    encode_angle(angles_deg[i], high, low);
    data.push_back(high);
    data.push_back(low);
  }
  data.push_back(static_cast<uint8_t>(clamped_speed));

  if (!send_command(CMD_SEND_ANGLES, data)) {
    return false;
  }

  // Deliberately do not wait for the 0x22 ack. It carries nothing we use, and the
  // persistent RX buffer now absorbs it: get_angles() skips non-matching frames and
  // discards everything preceding the reply it consumes. Blocking here previously
  // spent part of every cycle waiting on a frame that was then thrown away.
  return true;
}

bool MyCobotSerial::is_power_on()
{
  if (!impl_->port_open) {
    return false;
  }

  try {
    impl_->port.FlushInputBuffer();
  } catch (...) {}
  impl_->rx.clear();  // flushing the port must not leave stale carry-over bytes

  if (!send_command(CMD_IS_POWER_ON)) {
    return false;
  }

  uint8_t rx_cmd_id;
  std::vector<uint8_t> rx_payload;
  if (!read_response(rx_cmd_id, rx_payload, 1000, CMD_IS_POWER_ON)) {
    return false;
  }

  return (!rx_payload.empty() && rx_payload[0] == 1);
}

bool MyCobotSerial::release_all_servos()
{
  return send_command(CMD_RELEASE_ALL_SERVOS);
}

bool MyCobotSerial::focus_all_servos()
{
  // pymycobot sends FOCUS_SERVO (0x57) with servo_id=0 to mean "all servos"
  // (verified against pymycobot/generate.py lines 488–495, 2026-02-25)
  return send_command(CMD_FOCUS_SERVO, {0x00});
}

bool MyCobotSerial::set_gripper_value(int value, int speed, int gripper_type)
{
  // pymycobot set_gripper_value(value, speed, gripper_type) -> SET_GRIPPER_VALUE
  // (0x67) with each arg as a single byte. Confirmed on hardware 2026-08-05: the
  // adaptive gripper (gripper_type=1) opens at 100 and closes at 0.
  const int v = std::max(0, std::min(100, value));
  const int s = std::max(1, std::min(100, speed));
  const std::vector<uint8_t> data = {
    static_cast<uint8_t>(v),
    static_cast<uint8_t>(s),
    static_cast<uint8_t>(gripper_type),
  };
  // Fire-and-forget: no ack wait, matching send_angles(). The persistent RX buffer
  // in get_angles()/get_gripper_value() skips any interleaved 0x67 ack.
  return send_command(CMD_SET_GRIPPER_VALUE, data);
}

bool MyCobotSerial::get_gripper_value(int & value, int gripper_type)
{
  if (!impl_->port_open) {
    return false;
  }

  if (!send_command(CMD_GET_GRIPPER_VALUE, {static_cast<uint8_t>(gripper_type)})) {
    return false;
  }

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  size_t frame_end = 0;

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(QUERY_TIMEOUT_MS);

  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      uint8_t byte;
      impl_->port.ReadByte(byte, 5);
      impl_->rx.push_back(byte);
    } catch (...) {
      // No byte within the per-byte timeout; keep polling until the deadline.
    }
    if (find_last_frame(impl_->rx, CMD_GET_GRIPPER_VALUE, cmd_id, payload, frame_end)) {
      found = true;
      break;
    }
  }

  if (found) {
    impl_->rx.erase(impl_->rx.begin(), impl_->rx.begin() + frame_end);
  } else if (impl_->rx.size() > MAX_RX_BUFFER) {
    impl_->rx.erase(impl_->rx.begin(), impl_->rx.begin() + impl_->rx.size() / 2);
  }

  if (!found || payload.empty()) {
    return false;
  }
  value = static_cast<int>(payload[0]);
  return true;
}

// ---------------------------------------------------------------------------
// Static helpers — encoding / decoding / frame construction
// ---------------------------------------------------------------------------
void MyCobotSerial::encode_angle(double degrees, uint8_t & high, uint8_t & low)
{
  // pymycobot: int(angle * 100), then struct.pack(">h", value)
  int16_t value = static_cast<int16_t>(std::round(degrees * 100.0));
  high = static_cast<uint8_t>((value >> 8) & 0xFF);
  low  = static_cast<uint8_t>(value & 0xFF);
}

double MyCobotSerial::decode_angle(uint8_t high, uint8_t low)
{
  // pymycobot: struct.unpack(">h", data)[0], then round(value / 100.0, 3)
  int16_t value = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  return std::round((value / 100.0) * 1000.0) / 1000.0;
}

std::vector<uint8_t> MyCobotSerial::build_frame(
  uint8_t command_id, const std::vector<uint8_t> & data)
{
  // Frame: [0xFE][0xFE][LEN][CMD_ID][DATA...][0xFA]
  // LEN = 1 (cmd_id) + len(data) + 1 (footer)
  uint8_t length = static_cast<uint8_t>(1 + data.size() + 1);

  std::vector<uint8_t> frame;
  frame.reserve(4 + data.size());  // header(2) + length(1) + cmd(1) + data + footer(1)

  frame.push_back(HEADER);
  frame.push_back(HEADER);
  frame.push_back(length);
  frame.push_back(command_id);
  frame.insert(frame.end(), data.begin(), data.end());
  frame.push_back(FOOTER);

  return frame;
}

bool MyCobotSerial::parse_frame(
  const std::vector<uint8_t> & buffer,
  uint8_t & cmd_id,
  std::vector<uint8_t> & payload,
  int expected_cmd)
{
  if (buffer.size() < 5) {
    return false;
  }

  // Scan buffer for header [0xFE][0xFE]
  for (size_t i = 0; i + 4 < buffer.size(); ++i) {
    if (buffer[i] == HEADER && buffer[i + 1] == HEADER) {
      uint8_t length = buffer[i + 2];
      if (length < 2) {
        continue;
      }

      size_t expected_total = i + 3 + static_cast<size_t>(length);
      if (buffer.size() < expected_total) {
        // Not enough bytes accumulated yet for complete frame
        continue;
      }

      if (buffer[expected_total - 1] != FOOTER) {
        continue;
      }

      // Not the reply we asked for (e.g. a POWER_ON 0x10 frame arriving ahead of
      // the GET_ANGLES reply in the same burst) — keep scanning rather than
      // reporting this frame and discarding the rest of the buffer.
      if (expected_cmd >= 0 && buffer[i + 3] != static_cast<uint8_t>(expected_cmd)) {
        continue;
      }

      cmd_id = buffer[i + 3];

      size_t data_len = static_cast<size_t>(length) - 2;
      payload.clear();
      if (data_len > 0) {
        payload.assign(
          buffer.begin() + i + 4,
          buffer.begin() + i + 4 + data_len);
      }

      return true;
    }
  }

  return false;
}

bool MyCobotSerial::find_last_frame(
  const std::vector<uint8_t> & buffer,
  int expected_cmd,
  uint8_t & cmd_id,
  std::vector<uint8_t> & payload,
  size_t & frame_end)
{
  bool found = false;

  for (size_t i = 0; i + 4 < buffer.size(); ++i) {
    if (buffer[i] != HEADER || buffer[i + 1] != HEADER) {
      continue;
    }

    uint8_t length = buffer[i + 2];
    if (length < 2) {
      continue;
    }

    size_t expected_total = i + 3 + static_cast<size_t>(length);
    if (buffer.size() < expected_total || buffer[expected_total - 1] != FOOTER) {
      continue;
    }

    if (expected_cmd >= 0 && buffer[i + 3] != static_cast<uint8_t>(expected_cmd)) {
      continue;
    }

    // Keep going: a later frame in the buffer is fresher than this one.
    cmd_id = buffer[i + 3];
    size_t data_len = static_cast<size_t>(length) - 2;
    payload.assign(buffer.begin() + i + 4, buffer.begin() + i + 4 + data_len);
    frame_end = expected_total;
    found = true;
  }

  return found;
}

// ---------------------------------------------------------------------------
// Private — serial I/O
// ---------------------------------------------------------------------------
bool MyCobotSerial::send_command(uint8_t command_id, const std::vector<uint8_t> & data)
{
  if (!impl_->port_open) {
    return false;
  }

  auto frame = build_frame(command_id, data);

  try {
    for (uint8_t b : frame) {
      impl_->port.WriteByte(b);
    }
    impl_->port.DrainWriteBuffer();
  } catch (const std::exception &) {
    return false;
  }

  return true;
}

bool MyCobotSerial::read_response(
  uint8_t & cmd_id, std::vector<uint8_t> & payload, int timeout_ms, int expected_cmd)
{
  if (!impl_->port_open) {
    return false;
  }

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  std::vector<uint8_t> rx_buffer;
  rx_buffer.reserve(64);

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      uint8_t byte;
      impl_->port.ReadByte(byte, 20);
      rx_buffer.push_back(byte);
      if (parse_frame(rx_buffer, cmd_id, payload, expected_cmd)) {
        return true;
      }
    } catch (...) {
      // ReadByte timeout - continue polling until deadline
    }
  }

  return parse_frame(rx_buffer, cmd_id, payload, expected_cmd);
}

}  // namespace mycobot_hardware_interface
