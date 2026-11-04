#include "legend_chassis_adapter_can/can_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace legend_chassis_adapter_can
{
namespace can_protocol
{

namespace
{
constexpr uint32_t kTxCanId = 0x001;   // 下发控制
constexpr uint32_t kRxCanId = 0x010;   // 上行状态，按你 candump 的 010

constexpr uint8_t kDownId = 0x01;
constexpr uint8_t kUpId   = 0x02;

constexpr uint8_t kCmdMotion = 0x01;   // 速度 + 转向
constexpr uint8_t kCmdStatus = 0x02;   // 状态反馈
constexpr uint8_t kCmdBrake  = 0x04;   // 刹车 / 安全停车

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

constexpr double kMaxSpeedMps = 2.0;
constexpr double kMaxSteerDeg = 30.0;

inline double clampValue(double value, double min_value, double max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

inline can_frame makeEmptyTxFrame()
{
  can_frame frame {};
  frame.can_id = kTxCanId;
  frame.can_dlc = 8;
  for (int i = 0; i < 8; ++i) {
    frame.data[i] = 0x00;
  }
  return frame;
}

inline uint8_t clampBrake(float brake)
{
  if (brake >= 1.0f) {
    return 0xFF;  // 急刹车 / 全制动
  }
  if (brake <= 0.0f) {
    return 0x00;
  }

  int value = static_cast<int>(std::round(brake * 5.0f));
  if (value < 1) {
    value = 1;
  }
  if (value > 5) {
    value = 5;
  }
  return static_cast<uint8_t>(value);
}

}  // namespace

std::vector<can_frame> encodeVehicleCmd(
  const legend_interfaces::msg::VehicleCmd & cmd,
  double steering_offset_deg)
{
  std::vector<can_frame> frames;

  // 刹车优先
  if (cmd.brake > 0.0f) {
    can_frame brake_frame = makeEmptyTxFrame();
    brake_frame.data[0] = kDownId;
    brake_frame.data[1] = kCmdBrake;
    brake_frame.data[2] = clampBrake(cmd.brake);
    frames.push_back(brake_frame);
    return frames;
  }

  // 正常运动控制：01 01 + speed + steering
  can_frame motion_frame = makeEmptyTxFrame();
  motion_frame.data[0] = kDownId;
  motion_frame.data[1] = kCmdMotion;

  const double speed_mps =
    clampValue(static_cast<double>(cmd.velocity), -kMaxSpeedMps, kMaxSpeedMps);

  const double steering_deg =
    clampValue(
      static_cast<double>(cmd.steering) * kRadToDeg + steering_offset_deg,
      -kMaxSteerDeg, kMaxSteerDeg);

  const int16_t speed_raw =
    static_cast<int16_t>(std::round(speed_mps * 1000.0));

  const int16_t angle_raw =
    static_cast<int16_t>(std::round(steering_deg * 100.0));

  motion_frame.data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
  motion_frame.data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  motion_frame.data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  motion_frame.data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  motion_frame.data[6] = 0x00;
  motion_frame.data[7] = 0x00;

  frames.push_back(motion_frame);
  return frames;
}

bool decodeVehicleStatus(
  const struct can_frame & frame,
  legend_interfaces::msg::VehicleStatus & status,
  bool & estop_active)
{
  if (frame.can_id != kRxCanId) {
    return false;
  }

  if (frame.can_dlc < 8) {
    return false;
  }

  if (frame.data[0] != kUpId || frame.data[1] != kCmdStatus) {
    return false;
  }

  const int16_t raw_speed =
    static_cast<int16_t>(
      frame.data[2] |
      (static_cast<int16_t>(frame.data[3]) << 8));

  const int16_t raw_angle =
    static_cast<int16_t>(
      frame.data[4] |
      (static_cast<int16_t>(frame.data[5]) << 8));

  status.velocity = static_cast<float>(raw_speed) / 1000.0f;
  status.steering =
    static_cast<float>((static_cast<double>(raw_angle) / 100.0) * kDegToRad);

  status.battary = static_cast<float>(frame.data[6]);

  const uint8_t state = frame.data[7];
  estop_active = ((state >> 3) & 0x01) != 0;

  status.brake = 0.0f;
  status.gears = 0.0f;
  status.led = "no";

  return true;
}

}  // namespace can_protocol
}  // namespace legend_chassis_adapter_can 
