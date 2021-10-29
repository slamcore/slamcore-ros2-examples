#ifndef KOBUKI_JOYSTICK_H_
#define KOBUKI_JOYSTICK_H_

#include <linux/joystick.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/motor_power.hpp"

namespace kobuki_teleop_joy
{
class KobukiTeleopJoy: public rclcpp::Node
{
public:
  KobukiTeleopJoy();
  ~KobukiTeleopJoy();
  void spin();

private:
  bool readEvent(js_event& event);
  void enable();
  void disable();

private:
  // ROS Parameters
  std::string m_input_device{"/dev/input/js0"};
  float m_scale_linear{1.0f};
  float m_scale_angular{1.0f};
  std::string m_joystick_mode{"new"};

  // axis buttons of interest
  int m_btn_L3Y;
  int m_btn_R3X;

  int m_fd{0};
  bool m_enabled{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocity_publisher;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::MotorPower>::SharedPtr m_motor_power_publisher;
  std::shared_ptr<geometry_msgs::msg::Twist> m_twist_msg;
};

} // namespace kobuki_teleop_joy

#endif
