#ifndef LEGEND_CHASSIS_ADAPTER_CAN__CAN_PROTOCOL_HPP_
#define LEGEND_CHASSIS_ADAPTER_CAN__CAN_PROTOCOL_HPP_

#include <vector>
#include <linux/can.h>

#include "legend_interfaces/msg/vehicle_cmd.hpp"
#include "legend_interfaces/msg/vehicle_status.hpp"

namespace legend_chassis_adapter_can
{
namespace can_protocol
{

std::vector<can_frame> encodeVehicleCmd(
  const legend_interfaces::msg::VehicleCmd & cmd,
  double steering_offset_deg = 0.0);

bool decodeVehicleStatus(
  const struct can_frame & frame,
  legend_interfaces::msg::VehicleStatus & status,
  bool & estop_active);

}  // namespace can_protocol
}  // namespace legend_chassis_adapter_can

#endif  // LEGEND_CHASSIS_ADAPTER_CAN__CAN_PROTOCOL_HPP_

