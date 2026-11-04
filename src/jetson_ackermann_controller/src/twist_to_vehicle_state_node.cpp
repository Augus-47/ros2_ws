#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "car_msg/msg/vehicle_state.hpp"

static double rad2deg(double r) { return r * 180.0 / M_PI; }

class TwistToVehicleState : public rclcpp::Node
{
public:
  TwistToVehicleState()
  : Node("twist_to_vehicle_state")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 0.8);
    max_steer_deg_ = declare_parameter<double>("max_steer_deg", 45.0);
    steering_in_degrees_ = declare_parameter<bool>("steering_in_degrees", true);

    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    out_topic_ = declare_parameter<std::string>("out_topic", "/control_cmd");
    pub_hz_ = declare_parameter<double>("publish_hz", 10.0);

    gear_forward_ = declare_parameter<int>("gear_forward", 0);
    gear_reverse_ = declare_parameter<int>("gear_reverse", 1);

    led_ = declare_parameter<std::string>("led", "no");
    battery_ = declare_parameter<double>("battery", 90.0);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [&](const geometry_msgs::msg::Twist::SharedPtr msg){
        last_twist_ = *msg;
        got_ = true;
      });

    pub_ = create_publisher<car_msg::msg::VehicleState>(out_topic_, rclcpp::QoS(10));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pub_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TwistToVehicleState::onTimer, this));

    RCLCPP_INFO(get_logger(), "TwistToVehicleState: %s -> %s @ %.1f Hz",
                cmd_vel_topic_.c_str(), out_topic_.c_str(), pub_hz_);
  }

private:
  void onTimer()
  {
    if (!got_) return;

    const double v = last_twist_.linear.x;
    const double w = last_twist_.angular.z;

    // unicycle -> ackermann steering
    // delta = atan(L * kappa) where kappa = w / v
    double delta = 0.0;
    const double eps = 1e-3;
    if (std::fabs(v) > eps) {
      const double kappa = w / v;
      delta = std::atan(wheelbase_ * kappa);
    } else {
      delta = 0.0; // v太小就别乱打角（防抖）
    }

    const double max_delta = (max_steer_deg_ * M_PI / 180.0);
    delta = std::clamp(delta, -max_delta, max_delta);

    car_msg::msg::VehicleState out;
    out.velocity = v;
    out.steering = steering_in_degrees_ ? rad2deg(delta) : delta;

    out.brake = 0.0; // 你也可以做：需要急减速时根据(v_ref - v)给刹车
    out.gears = (v >= 0.0) ? gear_forward_ : gear_reverse_;
    out.led = led_;
    out.battary = battery_;

    pub_->publish(out);
  }

  // params
  double wheelbase_{0.8};
  double max_steer_deg_{45.0};
  bool steering_in_degrees_{true};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string out_topic_{"/control_cmd"};
  double pub_hz_{10.0};
  int gear_forward_{0};
  int gear_reverse_{1};
  std::string led_{"no"};
  double battery_{90.0};

  // io
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<car_msg::msg::VehicleState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_twist_;
  bool got_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToVehicleState>());
  rclcpp::shutdown();
  return 0;
}

