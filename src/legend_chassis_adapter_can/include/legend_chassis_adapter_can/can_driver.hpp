// SocketCAN 只管打开、读、写。

#ifndef LEGEND_CHASSIS_ADAPTER_CAN__CAN_DRIVER_HPP_
#define LEGEND_CHASSIS_ADAPTER_CAN__CAN_DRIVER_HPP_

#include <string>
#include <linux/can.h>

namespace legend_chassis_adapter_can
{

class CanDriver
{
public:
  CanDriver();
  ~CanDriver();

  bool open(const std::string & if_name);
  void close();
  bool isOpen() const;

  bool writeFrame(const struct can_frame & frame);
  bool readFrame(struct can_frame & frame);

private:
  int sock_;
};

}  // namespace legend_chassis_adapter_can

#endif

