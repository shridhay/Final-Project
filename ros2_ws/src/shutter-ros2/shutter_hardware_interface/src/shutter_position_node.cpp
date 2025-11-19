#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "shutter_hardware_interface/dynamixel_ros_control.h"
#include "shutter_hardware_interface/shutter_position_interface.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("shutter_position_interface");

  // --- Parameters (override via launch or CLI) ---
  const std::string default_port = "/dev/ttyUSB0";
  const int default_baud = 4000000;

  node->declare_parameter<std::string>("port_name", default_port);
  node->declare_parameter<int>("baud_rate", default_baud);
  node->declare_parameter<std::string>("dynamixel_info", "");
  // Note: use_joint_states_topic is declared by DynamixelController, so we don't declare it here
  // to avoid duplicate declaration error

  const std::string port_name = node->get_parameter("port_name").as_string();
  const uint32_t baud_rate = static_cast<uint32_t>(node->get_parameter("baud_rate").as_int());
  const std::string yaml_file = node->get_parameter("dynamixel_info").as_string();

  if (yaml_file.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "Parameter 'dynamixel_info' is empty. Set it to your YAML path (e.g., config/dynamixel_joints_position.yaml).");
    rclcpp::shutdown();
    return 1;
  }

  // --- Create controller objects ---
  auto dynamixel_controller = std::make_shared<DynamixelController>(node);

  // Initialize workbench (serial) and load configuration
  if (!dynamixel_controller->initWorkbench(port_name, baud_rate)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to init DynamixelWorkbench. Check port_name='%s' and baud_rate=%u.",
                 port_name.c_str(), baud_rate);
    rclcpp::shutdown();
    return 1;
  }

  if (!dynamixel_controller->getDynamixelsInfo(yaml_file)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse YAML at '%s'.", yaml_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Ping until servos respond (same spirit as ROS1 with throttled error)
  while (rclcpp::ok() && !dynamixel_controller->loadDynamixels()) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                          "Dynamixels not responding. Check IDs / baud / power. Retrying...");
    rclcpp::sleep_for(1s);
  }

  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 1;
  }

  if (!dynamixel_controller->initDynamixels()) {
    RCLCPP_FATAL(node->get_logger(),
                 "Failed to init Dynamixels. Check control table settings (see Robotis e-manual).");
    rclcpp::shutdown();
    return 1;
  }

  if (!dynamixel_controller->initControlItems()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to init control items.");
    rclcpp::shutdown();
    return 1;
  }

  if (!dynamixel_controller->initSDKHandlers()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to set Dynamixel SDK handlers.");
    rclcpp::shutdown();
    return 1;
  }

  dynamixel_controller->initPublisher();

  // Bridge-like interface (mirrors your ROS1 RobotHW behavior, without controller_manager)
  ShutterPositionInterface shutter(node, dynamixel_controller);

  // Subscribe to joint commands
  auto joint_command_callback = [&shutter, node](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      // Set target command - interpolation will handle smooth movement
      std::array<double, 4> target = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
      shutter.setTargetCommand(target);
      RCLCPP_DEBUG(node->get_logger(), "Received joint command target: [%.3f, %.3f, %.3f, %.3f]",
                   target[0], target[1], target[2], target[3]);
    } else {
      RCLCPP_WARN(node->get_logger(), "Received joint command with %zu elements, expected 4", msg->data.size());
    }
  };

  auto joint_command_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_group_controller/command",
    10,
    joint_command_callback
  );
  
  RCLCPP_INFO(node->get_logger(), "Subscribed to joint commands on topic: joint_group_controller/command");

  // --- Jog routine (same semantics as ROS1) ---
  {
    rclcpp::Rate jog_rate(5.0);  // 5 Hz
    shutter.read();
    while (rclcpp::ok() && !shutter.jog()) {
      shutter.write(false /* enforce_limits */);
      shutter.read();
      dynamixel_controller->publishCallback(); // keep /joint_states flowing during jog
      jog_rate.sleep();
      rclcpp::spin_some(node);
    }
    RCLCPP_DEBUG(node->get_logger(), "Finished jogging to resting pose.");
  }

  // --- Main control loop @100 Hz (same as ROS1) ---
  rclcpp::Rate loop_rate(100.0);
  auto last_time = node->get_clock()->now();

  while (rclcpp::ok()) {
    shutter.read();

    const auto now = node->get_clock()->now();
    const auto elapsed = now - last_time;  // rclcpp::Duration
    last_time = now;

    // Apply soft-position & simple velocity limits, then command servos
    shutter.write(true /* enforce_limits */, elapsed);

    // Publishes /joint_states and temperature (if enabled)
    dynamixel_controller->publishCallback();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
