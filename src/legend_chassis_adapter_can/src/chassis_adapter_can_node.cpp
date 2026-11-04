// 只负责做ROS接口
// 订阅 /vehicle/cmd、发布 /vehicle/status、定时发送、接收状态帧、命令超时保护

#include "legend_chassis_adapter_can/chassis_adapter_can_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include <linux/can.h>

#include "legend_chassis_adapter_can/can_protocol.hpp"

using namespace std::chrono_literals;

namespace legend_chassis_adapter_can
{

ChassisAdapterCanNode::ChassisAdapterCanNode()
: Node("chassis_adapter_can_node"),
  has_cmd_(false),
  can_opened_(false),
  timeout_stop_sent_(false),
  in_timeout_brake_(false),
  recovery_cycles_left_(0)
{
  this->declare_parameter<std::string>("can_if", "can0");
  this->declare_parameter<double>("tx_rate", 50.0);
  this->declare_parameter<double>("rx_rate", 200.0);
  this->declare_parameter<double>("cmd_timeout", 0.30);
  this->declare_parameter<double>("steering_offset_deg", 0.0);
  this->declare_parameter<bool>("publish_zero_on_timeout", true);

  can_if_ = this->get_parameter("can_if").as_string();
  tx_rate_ = this->get_parameter("tx_rate").as_double();
  rx_rate_ = this->get_parameter("rx_rate").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
  steering_offset_deg_ = this->get_parameter("steering_offset_deg").as_double();
  publish_zero_on_timeout_ = this->get_parameter("publish_zero_on_timeout").as_bool();

  cmd_sub_ = this->create_subscription<legend_interfaces::msg::VehicleCmd>(
    "/vehicle/cmd",
    20,
    std::bind(&ChassisAdapterCanNode::cmdCallback, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<legend_interfaces::msg::VehicleStatus>(
    "/vehicle/status",
    20);

  can_opened_ = can_driver_.open(can_if_);
  if (!can_opened_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open CAN interface: %s",
      can_if_.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Opened CAN interface: %s",
      can_if_.c_str());
  }

  const auto tx_period =
    std::chrono::duration<double>(1.0 / std::max(tx_rate_, 1.0));
  const auto rx_period =
    std::chrono::duration<double>(1.0 / std::max(rx_rate_, 1.0));

  tx_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(tx_period),
    std::bind(&ChassisAdapterCanNode::txTimerCallback, this));

  rx_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(rx_period),
    std::bind(&ChassisAdapterCanNode::rxTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Started chassis_adapter_can_node | can_if=%s tx_rate=%.1f rx_rate=%.1f cmd_timeout=%.3f steering_offset_deg=%.3f",
    can_if_.c_str(),
    tx_rate_,
    rx_rate_,
    cmd_timeout_,
    steering_offset_deg_);
}

void ChassisAdapterCanNode::cmdCallback(
  const legend_interfaces::msg::VehicleCmd::SharedPtr msg)
{
  
  RCLCPP_INFO(
    this->get_logger(),
    "Received /vehicle/cmd velocity=%.3f steering=%.3f brake=%.3f gears=%.3f led=%s",
    static_cast<double>(msg->velocity),
    static_cast<double>(msg->steering),
    static_cast<double>(msg->brake),
    static_cast<double>(msg->gears),
    msg->led.c_str());

  const bool was_timed_out = has_cmd_ && commandTimedOut();

  last_cmd_ = *msg;
  last_cmd_stamp_ = this->now();
  has_cmd_ = true;
  timeout_stop_sent_ = false;
  
  const bool non_zero_cmd =
    (std::fabs(static_cast<double>(msg->velocity)) > 1e-4) ||
    (std::fabs(static_cast<double>(msg->steering)) > 1e-4) ||
    (std::fabs(static_cast<double>(msg->brake)) > 1e-4);

  if ((in_timeout_brake_ || was_timed_out) && non_zero_cmd) {
    recovery_cycles_left_ = 5;   // 连续 5 个发送周期做恢复
  }
}

bool ChassisAdapterCanNode::commandTimedOut() const
{
  if (!has_cmd_) {
    return false;
  }

  const double age = (this->now() - last_cmd_stamp_).seconds();
  return age > cmd_timeout_;
}

legend_interfaces::msg::VehicleCmd ChassisAdapterCanNode::makeSafeStopCmd() const
{
  legend_interfaces::msg::VehicleCmd cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";

  cmd.velocity = 0.0f;
  cmd.steering = 0.0f;
  cmd.brake = 1.0f;
  cmd.gears = 0.0f;
  cmd.led = "no";

  return cmd;
}

void ChassisAdapterCanNode::txTimerCallback()
{
  if (!can_opened_) {
    return;
  }

  if (!has_cmd_) {
    return;
  }

  // ===== 1. 超时：持续发送安全刹停 =====
  if (commandTimedOut()) {
    if (!publish_zero_on_timeout_) {
      return;
    }

    const auto stop_cmd = makeSafeStopCmd();
    const auto stop_frames =
      can_protocol::encodeVehicleCmd(stop_cmd, steering_offset_deg_);

    if (stop_frames.empty()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encodeVehicleCmd returned empty frame list for safe stop");
      return;
    }

    if (!timeout_stop_sent_) {
      const double age = (this->now() - last_cmd_stamp_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "Vehicle command timeout (%.3fs > %.3fs), sending safe stop",
        age,
        cmd_timeout_);
      timeout_stop_sent_ = true;
    }

    in_timeout_brake_ = true;

    for (const auto & frame : stop_frames) {
      if (frame.data[1] == 0x04) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          500,
          "TX timeout brake id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
          frame.can_id,
          frame.data[0], frame.data[1], frame.data[2], frame.data[3],
          frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
      }

      if (!can_driver_.writeFrame(frame)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Failed to write timeout brake CAN frame to interface %s",
          can_if_.c_str());
        return;
      }
    }

    return;
  }

  // ===== 2. 恢复阶段：先释放刹车，再发档位和运动 =====
  if (in_timeout_brake_ && recovery_cycles_left_ > 0) {
    // 2.1 释放刹车：01 04 00 00 00 00 00 00
    struct can_frame release_frame {};
    release_frame.can_id = 0x001;
    release_frame.can_dlc = 8;
    release_frame.data[0] = 0x01;
    release_frame.data[1] = 0x04;
    release_frame.data[2] = 0x00;
    release_frame.data[3] = 0x00;
    release_frame.data[4] = 0x00;
    release_frame.data[5] = 0x00;
    release_frame.data[6] = 0x00;
    release_frame.data[7] = 0x00;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "TX brake release id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
      release_frame.can_id,
      release_frame.data[0], release_frame.data[1], release_frame.data[2], release_frame.data[3],
      release_frame.data[4], release_frame.data[5], release_frame.data[6], release_frame.data[7]);

    if (!can_driver_.writeFrame(release_frame)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Failed to write brake release CAN frame to interface %s",
        can_if_.c_str());
      return;
    }

    // 2.2 再发正常控制帧（要求 encodeVehicleCmd 已恢复成多帧版：gear + motion）
    const auto recover_frames =
      can_protocol::encodeVehicleCmd(last_cmd_, steering_offset_deg_);

    if (recover_frames.empty()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encodeVehicleCmd returned empty frame list during recovery");
      return;
    }

    for (const auto & frame : recover_frames) {
      if (frame.data[1] == 0x01) {
        const int16_t speed_raw =
          static_cast<int16_t>(frame.data[2] | (static_cast<int16_t>(frame.data[3]) << 8));
        const int16_t steer_raw =
          static_cast<int16_t>(frame.data[4] | (static_cast<int16_t>(frame.data[5]) << 8));

        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          500,
          "TX recovery motion id=0x%03X speed_raw=%d steer_raw=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
          frame.can_id,
          speed_raw,
          steer_raw,
          frame.data[0], frame.data[1], frame.data[2], frame.data[3],
          frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
      } else if (frame.data[1] == 0x03) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          500,
          "TX recovery gear id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
          frame.can_id,
          frame.data[0], frame.data[1], frame.data[2], frame.data[3],
          frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
      }

      if (!can_driver_.writeFrame(frame)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Failed to write recovery CAN frame to interface %s",
          can_if_.c_str());
        return;
      }
    }

    recovery_cycles_left_--;
    if (recovery_cycles_left_ <= 0) {
      in_timeout_brake_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery sequence finished, back to normal TX");
    }

    return;
  }

  // ===== 3. 正常发送 =====
  const auto frames =
    can_protocol::encodeVehicleCmd(last_cmd_, steering_offset_deg_);

  if (frames.empty()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "encodeVehicleCmd returned empty frame list");
    return;
  }

  for (const auto & frame : frames) {
    if (frame.data[1] == 0x01) {
      const int16_t speed_raw =
        static_cast<int16_t>(frame.data[2] | (static_cast<int16_t>(frame.data[3]) << 8));
      const int16_t steer_raw =
        static_cast<int16_t>(frame.data[4] | (static_cast<int16_t>(frame.data[5]) << 8));

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "TX motion id=0x%03X speed_raw=%d steer_raw=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        frame.can_id,
        speed_raw,
        steer_raw,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    } else if (frame.data[1] == 0x03) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "TX gear id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        frame.can_id,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    } else if (frame.data[1] == 0x04) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "TX brake id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        frame.can_id,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    }

    if (!can_driver_.writeFrame(frame)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Failed to write CAN frame to interface %s",
        can_if_.c_str());
      return;
    }
  }
}



void ChassisAdapterCanNode::rxTimerCallback()
{
  if (!can_opened_) {
    return;
  }

  constexpr int kMaxFramesPerTick = 20;

  for (int i = 0; i < kMaxFramesPerTick; ++i) {
    struct can_frame frame {};
    if (!can_driver_.readFrame(frame)) {
      break;
    }

    legend_interfaces::msg::VehicleStatus status;
    bool estop_active = false;

    if (!can_protocol::decodeVehicleStatus(frame, status, estop_active)) {
      continue;
    }

    status.header.stamp = this->now();
    status.header.frame_id = "base_link";
    status_pub_->publish(status);

    if (estop_active) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Received chassis emergency-stop feedback");
    }
  }
}


}  // namespace legend_chassis_adapter_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<legend_chassis_adapter_can::ChassisAdapterCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

