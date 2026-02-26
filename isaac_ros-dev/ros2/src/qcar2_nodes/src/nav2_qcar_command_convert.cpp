#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_interfaces/msg/boolean_leds.hpp"

using namespace std::chrono_literals;

class Nav2QCarConverter : public rclcpp::Node
{
public:
  Nav2QCarConverter()
  : Node("nav2_qcar2_converter")
  {
    // -------- Parameters (可运行时 ros2 param set 调) --------
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    this->declare_parameter<std::string>("motor_cmd_topic", "/qcar2_motor_speed_cmd");

    // Bicycle/Ackermann model
    this->declare_parameter<double>("wheelbase_L", 0.256);   // 轴距(m)，先用0.25，后面校准
    this->declare_parameter<double>("v_eps", 0.02);         // 低速阈值，避免除0
    this->declare_parameter<double>("steer_max", 0.45);     // 最大转角(rad)，先用0.45
    this->declare_parameter<double>("steer_bias", -0.05);     // 固定偏置(可选)
    this->declare_parameter<bool>("invert_steering", false);

    // Throttle mapping: throttle = k * v
    this->declare_parameter<double>("k_throttle", 4.0);     // 速度->油门增益（关键）
    this->declare_parameter<double>("throttle_max", 0.30);  // 油门限幅
    this->declare_parameter<bool>("invert_throttle", false);
    this->declare_parameter<bool>("allow_reverse", false);  // 是否允许倒车（先关掉可减少前后抖动）

    // Publish & safety
    this->declare_parameter<double>("publish_rate_hz", 50.0); // 50Hz 足够
    this->declare_parameter<double>("cmd_timeout_s", 0.3);    // 超时停

    // -------- Get parameters --------
    cmd_vel_topic_   = this->get_parameter("cmd_vel_topic").as_string();
    motor_cmd_topic_ = this->get_parameter("motor_cmd_topic").as_string();

    L_             = this->get_parameter("wheelbase_L").as_double();
    v_eps_         = this->get_parameter("v_eps").as_double();
    steer_max_     = this->get_parameter("steer_max").as_double();
    steer_bias_    = this->get_parameter("steer_bias").as_double();
    invert_steer_  = this->get_parameter("invert_steering").as_bool();

    k_throttle_       = this->get_parameter("k_throttle").as_double();
    throttle_max_     = this->get_parameter("throttle_max").as_double();
    invert_throttle_  = this->get_parameter("invert_throttle").as_bool();
    allow_reverse_    = this->get_parameter("allow_reverse").as_bool();

    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    cmd_timeout_s_   = this->get_parameter("cmd_timeout_s").as_double();

    // -------- ROS I/O --------
    command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      motor_cmd_topic_, rclcpp::QoS(10));

    nav2_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&Nav2QCarConverter::nav2_command_callback, this, std::placeholders::_1));

    // 发布频率：50Hz（把你原来的 1ms 改掉）
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Nav2QCarConverter::command_publish, this));

    // LED（不影响控制，保留；如果你不用可以删掉）
    timer2_ = this->create_wall_timer(33ms, std::bind(&Nav2QCarConverter::led_publish, this));

    got_cmd_ = false;
    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "nav2_qcar2_converter started. sub=%s pub=%s | L=%.3f, k_throttle=%.2f, steer_max=%.2f rad, rate=%.1f Hz",
      cmd_vel_topic_.c_str(), motor_cmd_topic_.c_str(), L_, k_throttle_, steer_max_, publish_rate_hz_);
  }

private:
  void nav2_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    nav2_v_ = msg->linear.x;      // m/s
    nav2_w_ = msg->angular.z;     // rad/s
    last_cmd_time_ = this->now();
    got_cmd_ = true;
  }

  void command_publish()
  {
    // 取最新命令；若超时则停
    const auto now = this->now();
    double v = nav2_v_;
    double w = nav2_w_;

    if (!got_cmd_ || (now - last_cmd_time_).seconds() > cmd_timeout_s_) {
      v = 0.0;
      w = 0.0;
    }

    // ---- Step4: ω(rad/s) -> δ(rad) (bicycle model) ----
    double steer = 0.0;
    if (std::fabs(v) > v_eps_) {
      steer = std::atan(L_ * w / v);
    } else {
      steer = 0.0; // 低速不按ω算转角，避免抖
    }

    steer += steer_bias_;
    steer = std::clamp(steer, -steer_max_, steer_max_);
    if (invert_steer_) steer = -steer;

    // ---- v(m/s) -> throttle (归一化/油门) ----
    double throttle = k_throttle_ * v;
    if (!allow_reverse_ && throttle < 0.0) throttle = 0.0;
    throttle = std::clamp(throttle, -throttle_max_, throttle_max_);
    if (invert_throttle_) throttle = -throttle;

    qcar2_interfaces::msg::MotorCommands motor_command;
    motor_command.motor_names = {"steering_angle", "motor_throttle"};
    motor_command.values = {steer, throttle};
    command_publisher_->publish(motor_command);
  }

  void led_publish()
  {
    // 只做显示，不影响控制；每次先清空避免灯状态“粘住”
    std::fill(std::begin(led_values_), std::end(led_values_), false);

    if (std::fabs(nav2_v_) > 1e-6) {
      // moving: headlights on
      led_values_[8]  = true;
      led_values_[9]  = true;
      led_values_[10] = true;
      led_values_[11] = true;
      led_values_[12] = true;
      led_values_[13] = true;

      if (nav2_w_ > 0.01) {
        led_values_[14] = true;
        led_values_[6]  = true;
      } else if (nav2_w_ < -0.01) {
        led_values_[15] = true;
        led_values_[7]  = true;
      }
    } else {
      // stop: brake lights
      led_values_[0] = true;
      led_values_[1] = true;
      led_values_[2] = true;
      led_values_[3] = true;
    }

    // 如果你需要发布 LED，把下面取消注释并创建 led_publisher_
    // qcar2_interfaces::msg::BooleanLeds led_commands;
    // led_commands.led_names = {...};  // 你原来的 names 列表
    // led_commands.values.assign(std::begin(led_values_), std::end(led_values_));
    // led_publisher_->publish(led_commands);
  }

private:
  // Topics
  std::string cmd_vel_topic_;
  std::string motor_cmd_topic_;

  // Params
  double L_{0.25};
  double v_eps_{0.02};
  double steer_max_{0.45};
  double steer_bias_{0.0};
  bool invert_steer_{false};

  double k_throttle_{4.0};
  double throttle_max_{0.30};
  bool invert_throttle_{false};
  bool allow_reverse_{false};

  double publish_rate_hz_{50.0};
  double cmd_timeout_s_{0.3};

  // State
  double nav2_v_{0.0};
  double nav2_w_{0.0};
  bool got_cmd_{false};
  rclcpp::Time last_cmd_time_{0,0,RCL_ROS_TIME};

  bool led_values_[16]{false};

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
  rclcpp::Publisher<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2QCarConverter>());
  rclcpp::shutdown();
  return 0;
}
