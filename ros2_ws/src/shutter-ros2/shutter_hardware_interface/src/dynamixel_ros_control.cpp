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

// Authors: Taehun Lim (Darby)
// Refactored for use with Shutter, the robot photographer, by Sasha Lew
// Ported to ROS 2 by <you>

#include "shutter_hardware_interface/dynamixel_ros_control.h"

using rclcpp::Node;
using rclcpp::Logger;

DynamixelController::DynamixelController(const rclcpp::Node::SharedPtr& node)
: node_(node),
  dxl_wb_(nullptr),
  is_joint_state_topic_(false),
  is_present_temperature_topic_(true),
  is_moving_(false)
{
  // Parameters (keep names the same as ROS1 where possible)
  is_joint_state_topic_        = node_->declare_parameter<bool>("use_joint_states_topic", true);
  is_present_temperature_topic_ = node_->declare_parameter<bool>("publish_temperature", true);

  dxl_wb_ = new DynamixelWorkbench;
}

DynamixelController::~DynamixelController()
{
  delete dxl_wb_;
  dxl_wb_ = nullptr;
}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log = nullptr;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (!result)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "dxl_wb_->init() failed");
  }
  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel = YAML::LoadFile(yaml_file.c_str());
  if (!dynamixel || dynamixel.IsNull())
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); ++it_file)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.empty())
      continue;

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); ++it_item)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      dynamixel_info_.emplace_back(name, item_value);
    }
  }
  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log = nullptr;

  for (const auto& dxl : dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping(static_cast<uint8_t>(dxl.second), &model_number, &log);
    if (!result)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "ping failed");
      RCLCPP_ERROR(node_->get_logger(), "Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Name : %s, ID : %u, Model Number : %u",
                  dxl.first.c_str(), dxl.second, model_number);
    }
  }
  return result;
}

bool DynamixelController::initDynamixels(void)
{
  const char* log = nullptr;

  for (const auto& dxl : dynamixel_)
  {
    for (const auto& info : dynamixel_info_)
    {
      if (dxl.first != info.first)
        continue;

      if (info.second.item_name == "ID" || info.second.item_name == "Baud_Rate")
        continue;

      // If Operating_Mode changes, torque off first
      if (info.second.item_name == "Operating_Mode")
      {
        int32_t operating_mode = 0;
        bool ok = dxl_wb_->itemRead(static_cast<uint8_t>(dxl.second),
                                    info.second.item_name.c_str(),
                                    &operating_mode, &log);
        if (!ok)
        {
          RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "itemRead failed");
          RCLCPP_FATAL(node_->get_logger(),
                       "Failed to read value [%d] on items [%s] to Dynamixel [Name: %s, ID: %u]",
                       info.second.value, info.second.item_name.c_str(),
                       dxl.first.c_str(), dxl.second);
          return false;
        }
        else
        {
          if (operating_mode != info.second.value)
            dxl_wb_->torqueOff(static_cast<uint8_t>(dxl.second));
          else
            continue;
        }
      }

      bool w = dxl_wb_->itemWrite(static_cast<uint8_t>(dxl.second),
                                  info.second.item_name.c_str(),
                                  info.second.value, &log);
      if (!w)
      {
        RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "itemWrite failed");
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %u]",
                     info.second.value, info.second.item_name.c_str(),
                     dxl.first.c_str(), dxl.second);
        return false;
      }
    }

    dxl_wb_->torqueOn(static_cast<uint8_t>(dxl.second));
  }

  return true;
}

bool DynamixelController::initControlItems(void)
{
  const char* log = nullptr;
  (void)log;

  auto it = dynamixel_.begin();
  if (it == dynamixel_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "No Dynamixel IDs loaded.");
    return false;
  }

  const ControlItem* goal_position    = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == nullptr) return false;

  const ControlItem* goal_velocity    = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == nullptr)
    goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == nullptr) return false;

  const ControlItem* present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == nullptr) return false;

  const ControlItem* present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == nullptr)
    present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == nullptr) return false;

  const ControlItem* present_current  = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == nullptr)
    present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == nullptr) return false;

  control_items_["Goal_Position"]    = goal_position;
  control_items_["Goal_Velocity"]    = goal_velocity;
  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"]  = present_current;

  return true;
}

bool DynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = nullptr;

  auto it = dynamixel_.begin();
  if (it == dynamixel_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "No Dynamixel IDs loaded.");
    return false;
  }

  result = dxl_wb_->addSyncWriteHandler(
      control_items_["Goal_Position"]->address,
      control_items_["Goal_Position"]->data_length,
      &log);
  if (!result)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "addSyncWriteHandler Goal_Position failed");
    return result;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "%s", log ? log : "addSyncWriteHandler Goal_Position OK");
  }

  result = dxl_wb_->addSyncWriteHandler(
      control_items_["Goal_Velocity"]->address,
      control_items_["Goal_Velocity"]->data_length,
      &log);
  if (!result)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "addSyncWriteHandler Goal_Velocity failed");
    return result;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "%s", log ? log : "addSyncWriteHandler Goal_Velocity OK");
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address =
      std::min(control_items_["Present_Position"]->address,
               control_items_["Present_Current"]->address);

    // As some models have a gap between Present_Velocity and Present_Current, +2 per your note
    uint16_t read_length =
      control_items_["Present_Position"]->data_length +
      control_items_["Present_Velocity"]->data_length +
      control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(start_address, read_length, &log);
    if (!result)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", log ? log : "addSyncReadHandler failed");
      return result;
    }
  }

  return result;
}

void DynamixelController::initPublisher(void)
{
  if (is_joint_state_topic_)
  {
    // Keep the same topic names; remap as needed at launch
    joint_states_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
  }
  if (is_present_temperature_topic_)
  {
    present_temperature_pub_ = node_->create_publisher<std_msgs::msg::Int32>("present_temperature", 100);
  }
}

void DynamixelController::initTemperaturePublisher(void)
{
  // kept for parity; initPublisher covers it
  if (!present_temperature_pub_ && is_present_temperature_topic_)
  {
    present_temperature_pub_ = node_->create_publisher<std_msgs::msg::Int32>("present_temperature", 100);
  }
}

void DynamixelController::readCallback()
{
  bool result = false;
  const char* log = nullptr;

  // Clear and prepare local state buffers
  dynamixel_state_list_.dynamixel_state.clear();

  const size_t N = dynamixel_.size();
  if (N == 0) return;

  std::vector<int32_t> get_current(N, 0);
  std::vector<int32_t> get_velocity(N, 0);
  std::vector<int32_t> get_position(N, 0);
  std::vector<uint8_t> id_array(N, 0);
  uint8_t id_cnt = 0;

  std::vector<dynamixel_workbench_msgs::msg::DynamixelState> dyn_states(N);

  for (const auto& dxl : dynamixel_)
  {
    dyn_states[id_cnt].name = dxl.first;
    dyn_states[id_cnt].id   = static_cast<uint8_t>(dxl.second);
    id_array[id_cnt]        = static_cast<uint8_t>(dxl.second);
    id_cnt++;
  }

  if (!is_moving_)
  {
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      result = dxl_wb_->syncRead(
          SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
          id_array.data(),
          static_cast<uint16_t>(N),
          &log);
      if (!result)
      {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "syncRead failed");
      }

      result = dxl_wb_->getSyncReadData(
          SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
          id_array.data(), id_cnt,
          control_items_["Present_Current"]->address,
          control_items_["Present_Current"]->data_length,
          get_current.data(), &log);
      if (!result)
      {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "getSyncReadData (current) failed");
      }

      result = dxl_wb_->getSyncReadData(
          SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
          id_array.data(), id_cnt,
          control_items_["Present_Velocity"]->address,
          control_items_["Present_Velocity"]->data_length,
          get_velocity.data(), &log);
      if (!result)
      {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "getSyncReadData (velocity) failed");
      }

      result = dxl_wb_->getSyncReadData(
          SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
          id_array.data(), id_cnt,
          control_items_["Present_Position"]->address,
          control_items_["Present_Position"]->data_length,
          get_position.data(), &log);
      if (!result)
      {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "getSyncReadData (position) failed");
      }

      for (uint8_t i = 0; i < id_cnt; ++i)
      {
        dyn_states[i].present_current  = get_current[i];
        dyn_states[i].present_velocity = get_velocity[i];
        dyn_states[i].present_position = get_position[i];
        dynamixel_state_list_.dynamixel_state.push_back(dyn_states[i]);
      }

      // Convert to rad/s and rad for your local buffers
      id_cnt = 0;
      for (const auto& dxl : dynamixel_)
      {
        double position = 0.0;
        double velocity = 0.0;

        velocity = dxl_wb_->convertValue2Velocity(
            static_cast<uint8_t>(dxl.second),
            static_cast<int32_t>(dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity));

        position = dxl_wb_->convertValue2Radian(
            static_cast<uint8_t>(dxl.second),
            static_cast<int32_t>(dynamixel_state_list_.dynamixel_state[id_cnt].present_position));

        pos_to_read[id_cnt] = position;
        vel_to_read[id_cnt] = velocity;
        id_cnt++;
      }
    }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
    {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Protocol 1.0 is unsupported; switch to Protocol 2.0");
    }
  }

  // Temperature on a specific ID (3), as in your original (kept)
  bool temperature_result = dxl_wb_->readRegister(3, "Present_Temperature", &temperature_data_, &log);
  if (!temperature_result)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "readRegister(Present_Temperature) failed");
  }
}

void DynamixelController::publishCallback()
{
  if (is_joint_state_topic_ && joint_states_pub_)
  {
    joint_state_msg_.header.stamp = node_->get_clock()->now();

    joint_state_msg_.name.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (const auto& dxl : dynamixel_)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort   = 0.0;

      joint_state_msg_.name.push_back(dxl.first);

      if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        // XL-320 uses load; others use current
        const auto& s = dynamixel_state_list_.dynamixel_state[id_cnt];
        if (std::strcmp(dxl_wb_->getModelName(static_cast<uint8_t>(dxl.second)), "XL-320") == 0)
          effort = dxl_wb_->convertValue2Load(static_cast<int16_t>(s.present_current));
        else
          effort = dxl_wb_->convertValue2Current(static_cast<int16_t>(s.present_current));
      }
      else if (dxl_wb_->getProtocolVersion() == 1.0f)
      {
        RCLCPP_ERROR(node_->get_logger(), "Protocol 1.0 is unsupported; upgrade firmware to Protocol 2.0");
      }

      const auto& s = dynamixel_state_list_.dynamixel_state[id_cnt];
      velocity = dxl_wb_->convertValue2Velocity(static_cast<uint8_t>(dxl.second),
                                                static_cast<int32_t>(s.present_velocity));
      position = dxl_wb_->convertValue2Radian(static_cast<uint8_t>(dxl.second),
                                              static_cast<int32_t>(s.present_position));

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_->publish(joint_state_msg_);
  }

  if (is_present_temperature_topic_ && present_temperature_pub_)
  {
    present_temperature_msg_.data = temperature_data_;
    present_temperature_pub_->publish(present_temperature_msg_);
  }
}

void DynamixelController::write(const double cmd[4])
{
  bool result = false;
  const char* log = nullptr;

  const size_t N = dynamixel_.size();
  if (N == 0) return;

  std::vector<uint8_t> id_array(N, 0);
  std::vector<int32_t> dynamixel_position(N, 0);

  // Your original assumed fixed order joint_1..joint_4
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4"};
  uint8_t id_cnt = 0;
  for (const auto& joint : joint_names)
  {
    id_array[id_cnt] = static_cast<uint8_t>(dynamixel_[joint]);
    id_cnt++;
  }

  for (uint8_t i = 0; i < id_cnt; ++i)
  {
    dynamixel_position[i] = dxl_wb_->convertRadian2Value(id_array[i], cmd[i]);
  }

  result = dxl_wb_->syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
      id_array.data(), id_cnt,
      dynamixel_position.data(), 1,
      &log);

  if (!result)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "syncWrite(Goal_Position) failed");
  }
}

void DynamixelController::writeVelocity(const double cmd[4])
{
  bool result = false;
  const char* log = nullptr;

  const size_t N = dynamixel_.size();
  if (N == 0) return;

  std::vector<uint8_t> id_array(N, 0);
  std::vector<int32_t> dynamixel_velocity(N, 0);

  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4"};
  uint8_t id_cnt = 0;
  for (const auto& joint : joint_names)
  {
    id_array[id_cnt] = static_cast<uint8_t>(dynamixel_[joint]);
    id_cnt++;
  }

  for (uint8_t i = 0; i < id_cnt; ++i)
  {
    dynamixel_velocity[i] = dxl_wb_->convertVelocity2Value(id_array[i], cmd[i]);
  }

  result = dxl_wb_->syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
      id_array.data(), id_cnt,
      dynamixel_velocity.data(), 1,
      &log);

  if (!result)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log ? log : "syncWrite(Goal_Velocity) failed");
  }
}
