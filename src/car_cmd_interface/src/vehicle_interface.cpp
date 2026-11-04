#include "car_cmd_interface/vehicle_interface.hpp"
#include <cmath>  // 只添加这个头文件

VehicleInterface::VehicleInterface() 
  : Node("vehicle_interface_node"), speed_str_("0000"), angle_str_("0000") {
  
    // 创建订阅器
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&VehicleInterface::vel_callback, this, std::placeholders::_1));

    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/angle", 10, std::bind(&VehicleInterface::angle_callback, this, std::placeholders::_1));

    cmd_sub_ = this->create_subscription<car_msg::msg::VehicleState>(
        "/control_cmd", 10, std::bind(&VehicleInterface::cmd_callback, this, std::placeholders::_1));

    // 创建发布器
    vehicle_info_pub_ = this->create_publisher<car_msg::msg::VehicleState>(
        "/vehicle_info", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
        std::bind(&VehicleInterface::timer_callback, this));


    // 初始化CAN总线连接
    {
        struct sockaddr_can addr;
        struct ifreq ifr;
        int s;
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        strcpy(ifr.ifr_name, "can0");
        ioctl(s, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        // 设置非阻塞模式
        int flags = fcntl(s, F_GETFL, 0);
        fcntl(s, F_SETFL, flags | O_NONBLOCK);
        
        if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("连接can0失败");
            return; 
        }
        sock_ = s;
    }
    
    // ★ 只添加必要的参数
    this->declare_parameter<double>("wheel_base", 0.8);          // 轴距
    this->declare_parameter<double>("steering_offset", 0.0);     // 转向偏置
    
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    steering_offset_ = this->get_parameter("steering_offset").as_double();
    
    //新加的，当nav2失败的时候，防止以原速行使，给一个stop
    this->declare_parameter<double>("cmd_timeout", 0.25);
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

    last_cmd_time_ = this->now();
    got_cmd_ = false;
    timeout_sent_ = false;

    
    RCLCPP_INFO(this->get_logger(), "车辆接口初始化完成");
}

// ★ 新增：简化的阿克曼转向计算
double VehicleInterface::calculate_steering_angle(double v, double w) {
    const double eps = 1e-3;
    if (std::fabs(v) < eps || std::fabs(w) < 1e-3) return 0.0;

    const double kappa = w / v;                         // 关键：用 w/v
    double steering_rad = std::atan(wheel_base_ * kappa);
    double steering_deg = steering_rad * 180.0 / M_PI;

    steering_deg += steering_offset_;

    // 限幅到±45（按你车的真实极限）
    steering_deg = std::clamp(steering_deg, -45.0, 45.0);
    return steering_deg;
}


void VehicleInterface::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    // 调试日志：一定要看这一行！
    RCLCPP_INFO(this->get_logger(), "Nav2 Cmd: %.3f", msg->linear.x);
    
    //新加的，当nav2失败的时候，防止以原速行使，给一个stop
    last_cmd_time_ = this->now();
    got_cmd_ = true;
    timeout_sent_ = false;   // 只要来新指令，就允许未来再次触发超时停车

    
    // 监听收到的速度信息
    double linear_x = msg->linear.x;
    
     // 1. 死区：太小就直接当 0
    const double deadband = 0.03;      // 根据车调，0.02~0.05 都可以试
    const double min_vel_cmd = 0.12;   // 最小实际下发速度，比如 0.1~0.15 m/s
    
    double cmd_v = 0.0;
    if (std::fabs(linear_x) < deadband) {
        cmd_v = 0.0;
    } else {
        cmd_v = std::copysign(std::max(std::fabs(linear_x), min_vel_cmd), linear_x);
    }
    
    
    int speed_abs = static_cast<int>(std::abs(cmd_v) * 1000);
    
    // 假设速度范围是 -327.67 m/s 到 327.67 m/s
    if (speed_abs > 32767) {
        speed_abs = 32767;
    }

    // 组合方向位和速度值
    uint16_t proto_speed = static_cast<uint16_t>(speed_abs);
    if (cmd_v < 0) {
        proto_speed |= 0x8000; // 设置第15位为1，表示后退
    }
    
    speed_str_ = int16_to_hex_string_little_endian(static_cast<int>(proto_speed));

    // ★ 修改：使用阿克曼转向计算替换简单比例
    double steer_angle = calculate_steering_angle(cmd_v, msg->angular.z);
    
    int current_angle = static_cast<int>(steer_angle * 100);
    angle_str_ = int16_to_hex_string_little_endian(current_angle);
    
    send_cmd();
}

void VehicleInterface::angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  // 监听收到的角度信息
  double angle_deg = msg->data + steering_offset_;  // ★ 添加偏置
  
  // 限幅
  if (angle_deg > 45.0) angle_deg = 45.0;
  if (angle_deg < -45.0) angle_deg = -45.0;
  
  int current_angle = static_cast<int>(angle_deg * 100);
  angle_str_ = int16_to_hex_string_little_endian(current_angle);
  
  send_cmd();
}

void VehicleInterface::cmd_callback(const car_msg::msg::VehicleState::SharedPtr msg) {
    // 监听收到的车辆控制命令
    double velocity = msg->velocity;
    int speed_abs = static_cast<int>(std::abs(velocity) * 1000);

    if (speed_abs > 32767) {
        speed_abs = 32767;
    }

    uint16_t proto_speed = static_cast<uint16_t>(speed_abs);
    if (velocity < 0) {
        proto_speed |= 0x8000; // 设置后退标志位
    }
    speed_str_ = int16_to_hex_string_little_endian(static_cast<int>(proto_speed));

    // ★ 修改：添加偏置
    double steering = msg->steering + steering_offset_;
    
    // 限幅
    if (steering > 45.0) steering = 45.0;
    if (steering < -45.0) steering = -45.0;
    
    int current_angle = static_cast<int>(steering * 100);
    angle_str_ = int16_to_hex_string_little_endian(current_angle);
    
    send_cmd();
}

// ★ 以下所有函数完全不变 ★

void VehicleInterface::send_cmd() {
   
    // 发送CAN命令
    std::string cmd_str = "0101" + speed_str_ + angle_str_ + "0000"; 
    std::vector<unsigned char> cmd_data;
    for (std::size_t i = 0; i < cmd_str.size(); i += 2) {
        std::string byte_str = cmd_str.substr(i, 2);
        unsigned char byte = std::stoi(byte_str, nullptr, 16);
        cmd_data.push_back(byte);
    }

    struct can_frame frame;
    frame.can_id = 0x00000001;
    frame.can_dlc = cmd_data.size();
    for (std::size_t i = 0; i < cmd_data.size(); i++) {
        frame.data[i] = cmd_data[i];
    }
    // 数据发送
    int nbytes = send(sock_, &frame, sizeof(struct can_frame), 0);
    
    // 发送结果显示
    if (nbytes == sizeof(struct can_frame)){
        RCLCPP_DEBUG_STREAM(this->get_logger(), 
            "已发送,速度：" << speed_str_ << "，角度：" << angle_str_);
    }else{
        RCLCPP_ERROR(this->get_logger(), "fail,发送给小车控制指令失败");
    }
}

void VehicleInterface::timer_callback() {
    // 从can读取数据，优化：读取所有可用数据
    struct can_frame frame;
    bool received_emergency_stop = false;
    
    while (true) {
        int nbytes = recv(sock_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
        
        // 如果没有更多数据可读或出错，则退出循环
        if (nbytes <= 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("recv error");
            }
            break;
        }
        
        // 处理接收到的数据
        process_can_frame(frame, received_emergency_stop);
    }
    
    //新加的，当nav2失败的时候，防止以原速行使，给一个stop
    // --- cmd_vel watchdog: timeout -> send stop ---
    if (got_cmd_) {
        const double age = (this->now() - last_cmd_time_).seconds();
        if (age > cmd_timeout_ && !timeout_sent_) {
            RCLCPP_WARN(this->get_logger(),
                    "cmd_vel timeout (%.3fs > %.3fs), sending STOP!",
                    age, cmd_timeout_);

            // 速度清零（建议转角也清零更安全）
            speed_str_ = int16_to_hex_string_little_endian(0);
            angle_str_ = int16_to_hex_string_little_endian(0);

            send_cmd();
            timeout_sent_ = true;
        }
    }
}

void VehicleInterface::process_can_frame(const struct can_frame& frame,
                                         bool& received_emergency_stop) 
{
    RCLCPP_INFO(this->get_logger(), "车辆速度");
    std::stringstream ss;
    for (int i = 0; i < frame.can_dlc; i++) {
        ss << std::hex << std::setfill('0') << std::setw(2)
           << static_cast<int>(frame.data[i]);
    }
    std::string message = ss.str();

    if (message.substr(0, 4) == "0202") {
        car_msg::msg::VehicleState vehicle_info;

        // ======== ★ 关键修改：用 int16_t 解析速度和角度 ========
        int16_t raw_speed = static_cast<int16_t>(
            (static_cast<uint16_t>(frame.data[3]) << 8) |
             static_cast<uint16_t>(frame.data[2]));

        int16_t raw_angle = static_cast<int16_t>(
            (static_cast<uint16_t>(frame.data[5]) << 8) |
             static_cast<uint16_t>(frame.data[4]));

        uint8_t battery_status = frame.data[6];   // 电量
        uint8_t flag           = frame.data[7];   // 标志位
        uint8_t bit_3          = (flag >> 3) & 0x01;  // 急停标志位

        // 单位换算：速度 1/1000 m/s，角度 1/100 deg
        double velocity = static_cast<double>(raw_speed) / 1000.0;
        double degree   = static_cast<double>(raw_angle) / 100.0;
        // ========================================================

        // 急停逻辑保持不变
        if (bit_3 == 1 && !received_emergency_stop) {
            received_emergency_stop = true;
            RCLCPP_WARN(this->get_logger(), "检测到急停信号！立即停止车辆");

            speed_str_ = int16_to_hex_string_little_endian(0);
            send_cmd();
        } else if (bit_3 == 0 && received_emergency_stop) {
            received_emergency_stop = false;
            RCLCPP_WARN(this->get_logger(), "急停解除，可以恢复运动");
        }

        // 车辆信息填充并发布
        rclcpp::Clock clock;
        vehicle_info.header.stamp    = clock.now();
        vehicle_info.header.frame_id = "base_link";

        // ★ 如果你发送时加了 +13° 偏置，这里要减回来（后面解释）
        //constexpr double STEER_OFFSET_DEG = 13.0;
        vehicle_info.steering = degree;

        vehicle_info.velocity = velocity;
        vehicle_info.gears    = 0;
        vehicle_info.led      = "no";
        vehicle_info.brake    = 0;
        vehicle_info.battary  = battery_status;

        RCLCPP_INFO(this->get_logger(),
                    "车辆速度：%.2f，角度：%.2f，电量: %f，急停位：%d",
                    vehicle_info.velocity,
                    vehicle_info.steering,
                    vehicle_info.battary,
                    static_cast<int>(bit_3));

        if (emergency_active_) {
            vehicle_info.velocity = 0.0;
            vehicle_info_pub_->publish(vehicle_info);
            return;
        }

        vehicle_info_pub_->publish(vehicle_info);
    }
}


// vehicle_interface.cpp
std::string VehicleInterface::int16_to_hex_string_little_endian(int int_value) {
  // 先收缩成 int16，再转成无符号方便按位操作
  int16_t value16 = static_cast<int16_t>(int_value);
  uint16_t u = static_cast<uint16_t>(value16);

  std::stringstream ss;
  // 小端：低字节在前，高字节在后
  ss << std::hex << std::setfill('0') << std::setw(2) << (u & 0xFF)
     << std::setw(2) << ((u >> 8) & 0xFF);
  return ss.str();
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
