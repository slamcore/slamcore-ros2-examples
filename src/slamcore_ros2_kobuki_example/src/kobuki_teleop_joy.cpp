#include "slamcore_ros2_kobuki_example/kobuki_teleop_joy.hpp"

#include <cstdint>
#include <fcntl.h>
#include <limits>
#include <unistd.h>

namespace kobuki_teleop_joy
{
enum Ds4Buttons
{
  CROSS,
  CIRCLE,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
};

enum class Ds4AxisOld: uint8_t
{
  L3_X=0,
  L3_Y,
  R3_X,
  R3_Y,
};

enum class Ds4AxisNew: uint8_t
{
  L3_X=0,
  L3_Y,
  L2,
  R3_X,
  R3_Y,
};


KobukiTeleopJoy::KobukiTeleopJoy() : Node("kobuki_teleop_joy"), m_twist_msg(new geometry_msgs::msg::Twist)
{
  RCLCPP_INFO(this->get_logger(), "Initializing %s", this->get_name());
  /* this->declare_parameter<std::string>("my_parameter", "world"); */
  this->declare_parameter<std::string>("input_device", m_input_device);
  this->declare_parameter<float>(std::string("scale_linear"), m_scale_linear);
  this->declare_parameter<float>(std::string("scale_angular"), m_scale_angular);
  this->declare_parameter<std::string>(std::string("joystick_mode"), m_joystick_mode);

  this->get_parameter_or("input_device", m_input_device, m_input_device);
  this->get_parameter_or("scale_linear", m_scale_linear, m_scale_linear);
  this->get_parameter_or("scale_angular", m_scale_angular, m_scale_angular);
  this->get_parameter_or("joystick_mode", m_joystick_mode, m_joystick_mode);

  if (m_joystick_mode == "new")
  {
    m_btn_L3Y = static_cast<uint8_t>(Ds4AxisNew::L3_Y);
    m_btn_R3X = static_cast<uint8_t>(Ds4AxisNew::R3_X);
  }
  else if (m_joystick_mode == "old")
  {
    RCLCPP_INFO(this->get_logger(), "Using alternative joystick mappings");
    m_btn_L3Y = static_cast<uint8_t>(Ds4AxisOld::L3_Y);
    m_btn_R3X = static_cast<uint8_t>(Ds4AxisOld::R3_X);
  }
  else
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "The joystick_mode parameter has to be set to either \"new\" or \"old\"");
    throw std::runtime_error("Initialization error");
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "KobukiTeleopJoy : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "KobukiTeleopJoy: Error opening joystick device \""
                                                << m_input_device << "\", is the joystick paired and connected?");
    throw std::runtime_error("Initialization error");
  }

  m_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("input/joy", 1);
  m_motor_power_publisher = this->create_publisher<kobuki_ros_interfaces::msg::MotorPower>("motor_power", 1);
}

void KobukiTeleopJoy::spin()
{
  constexpr float max = static_cast<float>(std::numeric_limits<int16_t>::max());

  js_event event;
  if (readEvent(event))
  {
    if (event.type == JS_EVENT_BUTTON && event.number == Ds4Buttons::L1)
    {
      if (!m_enabled && event.value == 1)
      {
        enable();
        m_enabled = true;
      }
      else if (m_enabled && event.value == 0)
      {
        disable();
        m_enabled = false;
      }
    }
    else if (event.type == JS_EVENT_AXIS)
    {
      if (event.number == m_btn_L3Y)
      {
        /* m_twist_msg->linear.x = -event.value / 32767.0 * m_scale_linear; */
        m_twist_msg->linear.x = -event.value / max * m_scale_linear;
      }
      else if (event.number == m_btn_R3X)
      {
        m_twist_msg->angular.z = -event.value / max * m_scale_angular;
      }
    }
  }

  if (m_enabled && (m_twist_msg->linear.x != 0 || m_twist_msg->angular.z != 0))
  {
    m_velocity_publisher->publish(*m_twist_msg);
  }
}

bool KobukiTeleopJoy::readEvent(js_event& event)
{
  if (read(m_fd, &event, sizeof(event)) == -1)
  {
    return false;
  }

  event.type &= ~JS_EVENT_INIT;

  return true;
}

void KobukiTeleopJoy::enable()
{
  kobuki_ros_interfaces::msg::MotorPower power_cmd;
  power_cmd.state = kobuki_ros_interfaces::msg::MotorPower::ON;
  m_motor_power_publisher->publish(power_cmd);
}

void KobukiTeleopJoy::disable()
{
  m_twist_msg->linear.x = 0;
  m_twist_msg->angular.z = 0;
  m_velocity_publisher->publish(*m_twist_msg);
  kobuki_ros_interfaces::msg::MotorPower power_cmd;
  power_cmd.state = kobuki_ros_interfaces::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(power_cmd);
}

KobukiTeleopJoy::~KobukiTeleopJoy() { RCLCPP_INFO(this->get_logger(), "Cave Johnson. We're Done Here"); }

} // namespace kobuki_teleop_joy

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<kobuki_teleop_joy::KobukiTeleopJoy>();
  while (rclcpp::ok())
  {
    node->spin();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
