#ifndef VEHICLE_INTERFACE_HPP
#define VEHICLE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <car_msg/msg/vehicle_state.hpp>


#include <cmath>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <fcntl.h>
#include <errno.h>

class VehicleInterface : public rclcpp::Node {
public:
  // 构造函数
  VehicleInterface();

private:

   //新加的，当nav2失败的时候，防止以原速行使，给一个stop
   double cmd_timeout_{0.25};
   rclcpp::Time last_cmd_time_;
   bool got_cmd_{false};
   bool timeout_sent_{false};


   double wheel_base_{0.0};
   double steering_offset_{0.0};

   double calculate_steering_angle(double v, double w);
  
   bool emergency_active_{false};
  
  // 回调函数(速度，角度，整体)
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
  //void send_brake_cmd(uint8_t brake_value);   // 需要刹车就加上
  void cmd_callback(const car_msg::msg::VehicleState::SharedPtr msg);
  
  // 定时器回调函数（发送车辆状态，接收CAN消息）
  void timer_callback();
  
  // 处理CAN帧数据
  void process_can_frame(const struct can_frame& frame, bool& received_emergency_stop);
  
  // 发送CAN命令
  void send_cmd();
  
  // 辅助函数：整数转换为小端十六进制字符串
  std::string int16_to_hex_string_little_endian(int int_value);
  
  // 订阅器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<car_msg::msg::VehicleState>::SharedPtr cmd_sub_;

  // 发布器
  rclcpp::Publisher<car_msg::msg::VehicleState>::SharedPtr vehicle_info_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 存储当前速度和角度的字符串
  int sock_; // can总线连接
  std::string speed_str_;
  std::string angle_str_;
  
  // ✅ 新增：记录上一次的速度与角速度，用于 send_cmd()
  double last_linear_x_ = 0.0;
  double last_angular_z_ = 0.0;
  

};

#endif // VEHICLE_INTERFACE_HPP
