/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
* Modifications for ROS 2 (Jazzy) by <you/your lab>, 2025.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*******************************************************************************/

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>
#include <dynamixel_workbench_msgs/msg/dynamixel_state.hpp>

// If you ported trajectory_generator to ROS 2, include it here as needed
// #include <dynamixel_workbench_controllers/trajectory_generator.hpp>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
private:
  // ROS 2 node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr         present_temperature_pub_;

  // Dynamixel Workbench
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::msg::DynamixelStateList dynamixel_state_list_;
  sensor_msgs::msg::JointState joint_state_msg_;

  bool is_joint_state_topic_;
  bool is_present_temperature_topic_;
  int32_t temperature_data_ = 0;
  std_msgs::msg::Int32 present_temperature_msg_;

  bool is_moving_;

public:
  explicit DynamixelController(const rclcpp::Node::SharedPtr& node);
  ~DynamixelController();

  double pos_to_read[4];
  double vel_to_read[4];

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);

  void initPublisher(void);
  void initTemperaturePublisher(void); // kept for parity

  void readCallback();
  void write(const double cmd[4]);
  void writeVelocity(const double cmd[4]);
  void publishCallback();
};

#endif // DYNAMIXEL_WORKBENCH_CONTROLLERS_H
