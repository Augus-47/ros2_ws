#ifndef LEGEND_CHASSIS_ADAPTER_CAN__CHASSIS_ADAPTER_CAN_NODE_HPP_
#define LEGEND_CHASSIS_ADAPTER_CAN__CHASSIS_ADAPTER_CAN_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "legend_interfaces/msg/vehicle_cmd.hpp"
#include "legend_interfaces/msg/vehicle_status.hpp"

#include "legend_chassis_adapter_can/can_driver.hpp"

namespace legend_chassis_adapter_can
{

class ChassisAdapterCanNode : public rclcpp::Node
{
public:
  ChassisAdapterCanNode();

private:
  void cmdCallback(const legend_interfaces::msg::VehicleCmd::SharedPtr msg);
  void txTimerCallback();
  void rxTimerCallback();

  bool commandTimedOut() const;
  legend_interfaces::msg::VehicleCmd makeSafeStopCmd() const;

  rclcpp::Subscription<legend_interfaces::msg::VehicleCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<legend_interfaces::msg::VehicleStatus>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;

  legend_interfaces::msg::VehicleCmd last_cmd_;
  rclcpp::Time last_cmd_stamp_;
  bool has_cmd_;

  std::string can_if_;
  double tx_rate_;
  double rx_rate_;
  double cmd_timeout_;
  double steering_offset_deg_;
  bool publish_zero_on_timeout_;

  bool can_opened_;
  bool timeout_stop_sent_;
  
  bool in_timeout_brake_;
  int recovery_cycles_left_;


  CanDriver can_driver_;
};

}  // namespace legend_chassis_adapter_can

#endif  // LEGEND_CHASSIS_ADAPTER_CAN__CHASSIS_ADAPTER_CAN_NODE_HPP_

