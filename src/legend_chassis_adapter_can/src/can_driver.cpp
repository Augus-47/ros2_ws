// 只负责底层SocketCAN  ----  打开can0, 读写帧，设备配置

#include "legend_chassis_adapter_can/can_driver.hpp"

#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace legend_chassis_adapter_can
{

CanDriver::CanDriver() : sock_(-1) {}

CanDriver::~CanDriver()
{
  close();
}

bool CanDriver::open(const std::string & if_name)
{
  close();

  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, if_name.c_str(), IFNAMSIZ - 1);

  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  return true;
}

void CanDriver::close()
{
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool CanDriver::isOpen() const
{
  return sock_ >= 0;
}

bool CanDriver::writeFrame(const struct can_frame & frame)
{
  if (sock_ < 0) {
    return false;
  }
  return ::write(sock_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CanDriver::readFrame(struct can_frame & frame)
{
  if (sock_ < 0) {
    return false;
  }

  int nbytes = ::recv(sock_, &frame, sizeof(frame), MSG_DONTWAIT);
  return nbytes == static_cast<int>(sizeof(frame));
}


}  // namespace legend_chassis_adapter_can

