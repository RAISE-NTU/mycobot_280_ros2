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

/// @file test_serial_protocol.cpp
/// @brief Unit tests for the myCobot serial protocol layer.
///
/// These tests verify encoding/decoding and frame construction against the
/// pymycobot reference implementation. No serial hardware is required.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "mycobot_hardware_interface/mycobot_serial.hpp"

using mycobot_hardware_interface::MyCobotSerial;

// ===========================================================================
// Angle Encoding Tests
// ===========================================================================

/// Verify encode/decode round-trip for a range of angles.
TEST(AngleEncoding, RoundTrip)
{
  // Test values spanning the typical myCobot joint range
  std::vector<double> test_angles = {
    0.0, 45.0, -45.0, 90.0, -90.0, 135.5, -135.5, 165.0, -165.0, 0.01, -0.01
  };

  for (double original : test_angles) {
    uint8_t high, low;
    MyCobotSerial::encode_angle(original, high, low);
    double decoded = MyCobotSerial::decode_angle(high, low);

    // Tolerance: ±0.01° due to integer quantisation (degrees × 100)
    EXPECT_NEAR(original, decoded, 0.01)
      << "Round-trip failed for angle " << original << "°";
  }
}

/// Verify specific encoding matches pymycobot reference output.
/// pymycobot: int(45.67 * 100) = 4567 = 0x11D7, packed as big-endian.
TEST(AngleEncoding, SpecificValues)
{
  {
    // 45.67° → int(45.67 * 100) = 4567 → 0x11D7
    uint8_t high, low;
    MyCobotSerial::encode_angle(45.67, high, low);
    EXPECT_EQ(high, 0x11);
    EXPECT_EQ(low,  0xD7);
  }

  {
    // 0° → 0x0000
    uint8_t high, low;
    MyCobotSerial::encode_angle(0.0, high, low);
    EXPECT_EQ(high, 0x00);
    EXPECT_EQ(low,  0x00);
  }

  {
    // -90° → int(-90 * 100) = -9000 → 0xDCD8 (two's complement)
    uint8_t high, low;
    MyCobotSerial::encode_angle(-90.0, high, low);
    int16_t expected = -9000;
    EXPECT_EQ(high, static_cast<uint8_t>((expected >> 8) & 0xFF));
    EXPECT_EQ(low,  static_cast<uint8_t>(expected & 0xFF));
  }

  {
    // 165° → int(165 * 100) = 16500 → 0x4074
    uint8_t high, low;
    MyCobotSerial::encode_angle(165.0, high, low);
    EXPECT_EQ(high, 0x40);
    EXPECT_EQ(low,  0x74);
  }
}

/// Verify decoding of specific byte pairs.
TEST(AngleEncoding, SpecificDecoding)
{
  // 0x11D7 → 4567 / 100.0 = 45.67
  EXPECT_NEAR(MyCobotSerial::decode_angle(0x11, 0xD7), 45.67, 0.001);

  // 0x0000 → 0.0
  EXPECT_DOUBLE_EQ(MyCobotSerial::decode_angle(0x00, 0x00), 0.0);

  // 0xFFFF → -1 / 100.0 = -0.01
  EXPECT_NEAR(MyCobotSerial::decode_angle(0xFF, 0xFF), -0.01, 0.001);

  // 0x8000 → -32768 / 100.0 = -327.68 (extreme boundary)
  EXPECT_NEAR(MyCobotSerial::decode_angle(0x80, 0x00), -327.68, 0.001);
}

/// Boundary values: max positive/negative int16.
TEST(AngleEncoding, BoundaryValues)
{
  // Maximum positive: 327.67° → 32767 → 0x7FFF
  {
    uint8_t high, low;
    MyCobotSerial::encode_angle(327.67, high, low);
    EXPECT_EQ(high, 0x7F);
    EXPECT_EQ(low,  0xFF);
  }

  // Maximum negative: -327.68° → -32768 → 0x8000
  {
    uint8_t high, low;
    MyCobotSerial::encode_angle(-327.68, high, low);
    EXPECT_EQ(high, 0x80);
    EXPECT_EQ(low,  0x00);
  }
}

// ===========================================================================
// Frame Construction Tests
// ===========================================================================

/// Verify GET_ANGLES frame matches expected byte sequence.
/// pymycobot builds: [0xFE, 0xFE, LEN, 0x20, 0xFA]
/// where LEN = 1 (cmd) + 0 (data) + 1 (footer) = 2
TEST(FrameConstruction, GetAngles)
{
  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_GET_ANGLES);

  std::vector<uint8_t> expected = {0xFE, 0xFE, 0x02, 0x20, 0xFA};
  EXPECT_EQ(frame, expected);
}

/// Verify RELEASE_ALL_SERVOS frame.
/// [0xFE, 0xFE, 0x02, 0x13, 0xFA]
TEST(FrameConstruction, ReleaseAllServos)
{
  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_RELEASE_ALL_SERVOS);

  std::vector<uint8_t> expected = {0xFE, 0xFE, 0x02, 0x13, 0xFA};
  EXPECT_EQ(frame, expected);
}

/// Verify FOCUS_SERVO (all servos) frame.
/// [0xFE, 0xFE, 0x03, 0x57, 0x00, 0xFA]
/// LEN = 1 (cmd) + 1 (data: servo_id=0) + 1 (footer) = 3
TEST(FrameConstruction, FocusAllServos)
{
  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_FOCUS_SERVO, {0x00});

  std::vector<uint8_t> expected = {0xFE, 0xFE, 0x03, 0x57, 0x00, 0xFA};
  EXPECT_EQ(frame, expected);
}

/// Verify SEND_ANGLES frame with known angles and speed.
/// angles: [0.0, 45.0, -90.0, 0.0, 0.0, 0.0], speed=50
/// Data: 6 × int16 big-endian + 1 byte speed
/// = [0x00,0x00, 0x11,0x94, 0xDC,0xE8, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x32]
/// LEN = 1 (cmd) + 13 (data) + 1 (footer) = 15 = 0x0F
TEST(FrameConstruction, SendAngles)
{
  // Build data manually: encode each angle then append speed
  std::vector<uint8_t> data;
  double angles[] = {0.0, 45.0, -90.0, 0.0, 0.0, 0.0};
  for (double angle : angles) {
    uint8_t h, l;
    MyCobotSerial::encode_angle(angle, h, l);
    data.push_back(h);
    data.push_back(l);
  }
  data.push_back(50);  // speed

  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_SEND_ANGLES, data);

  // Verify frame structure
  EXPECT_EQ(frame[0], 0xFE);                    // header
  EXPECT_EQ(frame[1], 0xFE);                    // header
  EXPECT_EQ(frame[2], 15);                      // length = 1 + 13 + 1
  EXPECT_EQ(frame[3], MyCobotSerial::CMD_SEND_ANGLES);  // 0x22
  EXPECT_EQ(frame.back(), 0xFA);                // footer

  // Verify total frame size: 2 (header) + 1 (len) + 15 (payload+footer) = 18
  EXPECT_EQ(frame.size(), 18u);

  // Verify angle bytes for joint 2 (45.0° → 4500 = 0x1194)
  EXPECT_EQ(frame[6], 0x11);   // high byte
  EXPECT_EQ(frame[7], 0x94);   // low byte

  // Verify angle bytes for joint 3 (-90.0° → -9000)
  int16_t neg90 = -9000;
  EXPECT_EQ(frame[8],  static_cast<uint8_t>((neg90 >> 8) & 0xFF));
  EXPECT_EQ(frame[9],  static_cast<uint8_t>(neg90 & 0xFF));

  // Verify speed byte (last data byte before footer)
  EXPECT_EQ(frame[16], 50);
}

// ===========================================================================
// Frame Parsing Tests
// ===========================================================================

/// Parse a valid GET_ANGLES response frame.
/// Simulated response: [0xFE, 0xFE, LEN, 0x20, <12 bytes>, 0xFA]
/// LEN = 1 (cmd) + 12 (data) + 1 (footer) = 14
TEST(FrameParsing, ValidGetAnglesResponse)
{
  // Simulate response with angles: [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
  // Encoded: 1000, 2000, 3000, 4000, 5000, 6000
  std::vector<uint8_t> buffer = {
    0xFE, 0xFE,  // header
    0x0E,        // length = 14
    0x20,        // cmd_id = GET_ANGLES
    // 10.0° = 1000 = 0x03E8
    0x03, 0xE8,
    // 20.0° = 2000 = 0x07D0
    0x07, 0xD0,
    // 30.0° = 3000 = 0x0BB8
    0x0B, 0xB8,
    // 40.0° = 4000 = 0x0FA0
    0x0F, 0xA0,
    // 50.0° = 5000 = 0x1388
    0x13, 0x88,
    // 60.0° = 6000 = 0x1770
    0x17, 0x70,
    0xFA         // footer
  };

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  ASSERT_TRUE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));

  EXPECT_EQ(cmd_id, MyCobotSerial::CMD_GET_ANGLES);
  ASSERT_EQ(payload.size(), 12u);

  // Decode and verify each angle
  double expected_angles[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
  for (size_t i = 0; i < 6; ++i) {
    double decoded = MyCobotSerial::decode_angle(payload[i * 2], payload[i * 2 + 1]);
    EXPECT_NEAR(decoded, expected_angles[i], 0.01)
      << "Joint " << i << " angle mismatch";
  }
}

/// Parse a minimal valid frame (no data, just cmd + footer).
TEST(FrameParsing, MinimalFrame)
{
  // A response to RELEASE_ALL_SERVOS might be: [0xFE, 0xFE, 0x02, 0x13, 0xFA]
  std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x02, 0x13, 0xFA};

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  ASSERT_TRUE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));

  EXPECT_EQ(cmd_id, 0x13);
  EXPECT_TRUE(payload.empty());
}

/// Reject a frame that is too short.
TEST(FrameParsing, TooShort)
{
  std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x02, 0x20};  // missing footer

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Reject a frame with wrong header.
TEST(FrameParsing, BadHeader)
{
  std::vector<uint8_t> buffer = {0xAA, 0xBB, 0x02, 0x20, 0xFA};

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Reject a frame with wrong footer.
TEST(FrameParsing, BadFooter)
{
  std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x02, 0x20, 0xFF};

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Reject a frame where length claims more data than available.
TEST(FrameParsing, LengthExceedsBuffer)
{
  // Length says 14 bytes follow, but only 2 are present
  std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x0E, 0x20, 0xFA};

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Reject a frame where length byte is impossibly small (< 2).
TEST(FrameParsing, LengthTooSmall)
{
  std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x01, 0x20, 0xFA};

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Empty buffer should fail gracefully.
TEST(FrameParsing, EmptyBuffer)
{
  std::vector<uint8_t> buffer;

  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
}

/// Regression: a lagging POWER_ON (0x10) frame arriving ahead of the GET_ANGLES
/// reply must not mask it.
///
/// Captured verbatim from a myCobot 280 M5 on /dev/ttyACM0 (2026-07-21). The
/// firmware emits both frames in a single 23-byte burst, after get_angles() has
/// already flushed the input buffer, so the unwanted frame cannot be flushed away.
/// Without an expected_cmd filter, parse_frame() returned the 0x10 frame and
/// get_angles() discarded the valid angle data sitting behind it — every read
/// failed regardless of timeout.
TEST(FrameParsing, InterleavedFrameDoesNotMaskExpectedReply)
{
  const std::vector<uint8_t> buffer = {
    0xFE, 0xFE, 0x03, 0x10, 0x01, 0xFA,                    // POWER_ON reply
    0xFE, 0xFE, 0x0E, 0x20,                                // GET_ANGLES reply
    0x00, 0x46, 0x00, 0x11, 0xFB, 0x0F,
    0xCA, 0xDB, 0xEB, 0xD9, 0xFF, 0xCC, 0xFA};

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;

  // Unfiltered: returns the first well-formed frame (the 0x10 one).
  ASSERT_TRUE(MyCobotSerial::parse_frame(buffer, cmd_id, payload));
  EXPECT_EQ(cmd_id, 0x10);

  // Filtered: skips past it and finds the reply we actually asked for.
  ASSERT_TRUE(MyCobotSerial::parse_frame(
      buffer, cmd_id, payload, MyCobotSerial::CMD_GET_ANGLES));
  EXPECT_EQ(cmd_id, MyCobotSerial::CMD_GET_ANGLES);
  ASSERT_EQ(payload.size(), MyCobotSerial::NUM_JOINTS * 2);

  // Same angles the live arm reported, to 2 dp.
  const std::vector<double> expected = {0.70, 0.17, -12.65, -136.05, -51.59, -0.52};
  for (size_t i = 0; i < MyCobotSerial::NUM_JOINTS; ++i) {
    EXPECT_NEAR(
      MyCobotSerial::decode_angle(payload[i * 2], payload[i * 2 + 1]),
      expected[i], 1e-9) << "joint " << i;
  }
}

/// A filtered parse must fail (not fall back to some other frame) when the
/// expected command is absent from the buffer.
TEST(FrameParsing, ExpectedCmdAbsentFails)
{
  const std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x03, 0x10, 0x01, 0xFA};

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  EXPECT_FALSE(MyCobotSerial::parse_frame(
      buffer, cmd_id, payload, MyCobotSerial::CMD_GET_ANGLES));
}

/// find_last_frame() must return the FRESHEST matching reply, not the first.
///
/// When the firmware lags, the persistent RX buffer holds a stale GET_ANGLES reply
/// followed by a current one. Returning the first would feed ros2_control joint angles
/// that get progressively older every cycle.
TEST(FrameScanning, ReturnsFreshestMatchingFrame)
{
  std::vector<uint8_t> buffer = {
    // stale reply: all joints at 1.00 deg
    0xFE, 0xFE, 0x0E, 0x20,
    0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0xFA,
    // a SEND_ANGLES ack landing between the two
    0xFE, 0xFE, 0x02, 0x22, 0xFA,
    // current reply: all joints at 2.00 deg
    0xFE, 0xFE, 0x0E, 0x20,
    0x00, 0xC8, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0xC8, 0xFA};

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  size_t frame_end = 0;

  ASSERT_TRUE(MyCobotSerial::find_last_frame(
      buffer, MyCobotSerial::CMD_GET_ANGLES, cmd_id, payload, frame_end));
  EXPECT_EQ(cmd_id, MyCobotSerial::CMD_GET_ANGLES);
  ASSERT_EQ(payload.size(), MyCobotSerial::NUM_JOINTS * 2);

  // 2.00 deg, i.e. the second frame — not the stale 1.00 deg one.
  EXPECT_NEAR(MyCobotSerial::decode_angle(payload[0], payload[1]), 2.00, 1e-9);

  // frame_end must cover the whole buffer so the caller consumes the ack too.
  EXPECT_EQ(frame_end, buffer.size());
}

/// A reply that is still incomplete must not be reported, and must not consume the
/// partial bytes — they are the start of the frame the next cycle will finish.
TEST(FrameScanning, IgnoresIncompleteTrailingFrame)
{
  std::vector<uint8_t> buffer = {
    0xFE, 0xFE, 0x0E, 0x20,
    0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0xFA,
    0xFE, 0xFE, 0x0E, 0x20, 0x00, 0xC8};   // truncated mid-frame

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  size_t frame_end = 0;

  ASSERT_TRUE(MyCobotSerial::find_last_frame(
      buffer, MyCobotSerial::CMD_GET_ANGLES, cmd_id, payload, frame_end));
  // Falls back to the complete 1.00 deg frame, consuming only that far.
  EXPECT_NEAR(MyCobotSerial::decode_angle(payload[0], payload[1]), 1.00, 1e-9);
  EXPECT_EQ(frame_end, 17u);
}

/// No matching frame at all must report failure rather than a stray match.
TEST(FrameScanning, NoMatchReportsFailure)
{
  const std::vector<uint8_t> buffer = {0xFE, 0xFE, 0x02, 0x22, 0xFA};

  uint8_t cmd_id = 0;
  std::vector<uint8_t> payload;
  size_t frame_end = 0;
  EXPECT_FALSE(MyCobotSerial::find_last_frame(
      buffer, MyCobotSerial::CMD_GET_ANGLES, cmd_id, payload, frame_end));
}

// ===========================================================================
// Cross-validation with pymycobot reference
// ===========================================================================

/// Cross-validate a full SEND_ANGLES frame against pymycobot output.
///
/// pymycobot builds send_angles([0, 45, -90, 135.5, -0.01, 180], 50) as:
///   angles_int = [0, 4500, -9000, 13550, -1, 18000]
///   encoded int16 big-endian: [0x0000, 0x1194, 0xDCE8, 0x34EE, 0xFFFF, 0x4650]
///   Frame: [0xFE, 0xFE, 0x0F, 0x22, data..., 0x32, 0xFA]
TEST(CrossValidation, SendAnglesFrame)
{
  double angles[] = {0.0, 45.0, -90.0, 135.5, -0.01, 180.0};

  std::vector<uint8_t> data;
  for (double angle : angles) {
    uint8_t h, l;
    MyCobotSerial::encode_angle(angle, h, l);
    data.push_back(h);
    data.push_back(l);
  }
  data.push_back(50);

  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_SEND_ANGLES, data);

  // Expected frame (derived from pymycobot _angle2int + _encode_int16 + _mesg)
  std::vector<uint8_t> expected = {
    0xFE, 0xFE,        // header
    0x0F,              // length = 15
    0x22,              // SEND_ANGLES
    0x00, 0x00,        // 0.0°   → 0
    0x11, 0x94,        // 45.0°  → 4500
    0xDC, 0xD8,        // -90.0° → -9000
    0x34, 0xEE,        // 135.5° → 13550
    0xFF, 0xFF,        // -0.01° → -1
    0x46, 0x50,        // 180.0° → 18000
    0x32,              // speed = 50
    0xFA               // footer
  };

  EXPECT_EQ(frame, expected)
    << "SEND_ANGLES frame does not match pymycobot reference output";
}

/// Cross-validate angle decoding for a simulated GET_ANGLES response.
TEST(CrossValidation, GetAnglesDecoding)
{
  // Simulated response data (6 × int16 big-endian):
  // angles_int = [0, 4500, -9000, 13550, -1, 18000]
  std::vector<uint8_t> response_data = {
    0x00, 0x00,  // 0.0°
    0x11, 0x94,  // 45.0°
    0xDC, 0xD8,  // -90.0°
    0x34, 0xEE,  // 135.5°
    0xFF, 0xFF,  // -0.01°
    0x46, 0x50,  // 180.0°
  };

  double expected[] = {0.0, 45.0, -90.0, 135.5, -0.01, 180.0};

  for (size_t i = 0; i < 6; ++i) {
    double decoded = MyCobotSerial::decode_angle(
      response_data[i * 2], response_data[i * 2 + 1]);
    EXPECT_NEAR(decoded, expected[i], 0.01)
      << "Joint " << i << ": expected " << expected[i] << "°, got " << decoded << "°";
  }
}

/// Verify the full 6-joint round-trip: encode all → build frame → parse → decode.
TEST(CrossValidation, FullRoundTrip)
{
  double original_angles[] = {0.0, 45.0, -90.0, 135.5, -0.01, 180.0};

  // Encode: build SEND_ANGLES frame
  std::vector<uint8_t> data;
  for (double angle : original_angles) {
    uint8_t h, l;
    MyCobotSerial::encode_angle(angle, h, l);
    data.push_back(h);
    data.push_back(l);
  }
  data.push_back(50);

  auto frame = MyCobotSerial::build_frame(MyCobotSerial::CMD_SEND_ANGLES, data);

  // Parse the frame back
  uint8_t cmd_id;
  std::vector<uint8_t> payload;
  ASSERT_TRUE(MyCobotSerial::parse_frame(frame, cmd_id, payload));
  EXPECT_EQ(cmd_id, MyCobotSerial::CMD_SEND_ANGLES);

  // Payload should be 13 bytes (12 angle bytes + 1 speed byte)
  ASSERT_EQ(payload.size(), 13u);

  // Decode the 6 angles from the parsed payload
  for (size_t i = 0; i < 6; ++i) {
    double decoded = MyCobotSerial::decode_angle(payload[i * 2], payload[i * 2 + 1]);
    EXPECT_NEAR(decoded, original_angles[i], 0.01)
      << "Round-trip failed for joint " << i;
  }

  // Verify speed byte
  EXPECT_EQ(payload[12], 50);
}

// ===========================================================================
// Protocol Constants Verification
// ===========================================================================

/// Verify protocol constants match pymycobot/common.py ProtocolCode values.
TEST(ProtocolConstants, MatchPymycobot)
{
  EXPECT_EQ(MyCobotSerial::HEADER, 0xFE);
  EXPECT_EQ(MyCobotSerial::FOOTER, 0xFA);
  EXPECT_EQ(MyCobotSerial::CMD_GET_ANGLES, 0x20);
  EXPECT_EQ(MyCobotSerial::CMD_SEND_ANGLES, 0x22);
  EXPECT_EQ(MyCobotSerial::CMD_RELEASE_ALL_SERVOS, 0x13);
  EXPECT_EQ(MyCobotSerial::CMD_FOCUS_SERVO, 0x57);
}
