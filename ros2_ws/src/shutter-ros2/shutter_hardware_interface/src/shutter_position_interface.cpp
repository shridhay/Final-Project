#include "shutter_hardware_interface/shutter_position_interface.h"

ShutterPositionInterface::ShutterPositionInterface(
    const rclcpp::Node::SharedPtr& node,
    std::shared_ptr<DynamixelController>& dxl_wb_ptr)
: node_(node),
  dynamixel_controller_(dxl_wb_ptr),
  first_read_initialized_(false)
{
  init_limits_hardcoded_();
  // Initialize cmd_ to match target_cmd_ initially
  cmd_ = target_cmd_;
  RCLCPP_INFO(node_->get_logger(), "ShutterPositionInterface: limits initialized and controller bound.");
}

void ShutterPositionInterface::setTargetCommand(const std::array<double, 4>& target)
{
  target_cmd_ = target;
  RCLCPP_DEBUG(node_->get_logger(), "Target command set: [%.3f, %.3f, %.3f, %.3f]",
               target[0], target[1], target[2], target[3]);
}

void ShutterPositionInterface::init_limits_hardcoded_()
{
  // ---- Joint 1 ----
  limits_[0].hard.has_velocity_limits = true;
  limits_[0].hard.max_velocity        = 0.785;
  limits_[0].hard.has_position_limits = true;
  limits_[0].hard.max_position        = 2.617;
  limits_[0].hard.min_position        = -2.617;

  limits_[0].soft.k_position   = 1.0;
  limits_[0].soft.max_position = 2.61;
  limits_[0].soft.min_position = -2.61;
  limits_[0].has_soft = true;

  // ---- Joint 2 ----
  limits_[1].hard.has_velocity_limits = true;
  limits_[1].hard.max_velocity        = 1.571;
  limits_[1].hard.has_position_limits = true;
  limits_[1].hard.max_position        = 1.571;
  limits_[1].hard.min_position        = -1.571;

  limits_[1].soft.k_position   = 5.0;
  limits_[1].soft.max_position = 1.54;
  limits_[1].soft.min_position = -1.54;
  limits_[1].has_soft = true;

  // ---- Joint 3 ----
  limits_[2].hard.has_velocity_limits = true;
  limits_[2].hard.max_velocity        = 1.571;
  limits_[2].hard.has_position_limits = true;
  limits_[2].hard.max_position        = 1.571;
  limits_[2].hard.min_position        = -1.571;

  limits_[2].soft.k_position   = 5.0;
  limits_[2].soft.max_position = 1.54;
  limits_[2].soft.min_position = -1.54;
  limits_[2].has_soft = true;

  // ---- Joint 4 ----
  limits_[3].hard.has_velocity_limits = true;
  limits_[3].hard.max_velocity        = 1.571;
  limits_[3].hard.has_position_limits = true;
  limits_[3].hard.max_position        = 1.571;
  limits_[3].hard.min_position        = -1.571;

  limits_[3].soft.k_position   = 1.0;
  limits_[3].soft.max_position = 1.54;
  limits_[3].soft.min_position = -1.54;
  limits_[3].has_soft = true;
}

void ShutterPositionInterface::read()
{
  // Mirror ROS1 semantics: pull from DynamixelController, shove into pos_/vel_/eff_
  dynamixel_controller_->readCallback();

  pos_[0] = dynamixel_controller_->pos_to_read[0];
  pos_[1] = dynamixel_controller_->pos_to_read[1];
  pos_[2] = dynamixel_controller_->pos_to_read[2];
  pos_[3] = dynamixel_controller_->pos_to_read[3];

  vel_[0] = dynamixel_controller_->vel_to_read[0];
  vel_[1] = dynamixel_controller_->vel_to_read[1];
  vel_[2] = dynamixel_controller_->vel_to_read[2];
  vel_[3] = dynamixel_controller_->vel_to_read[3];

  // Initialize cmd_ and target_cmd_ to current position on first read
  if (!first_read_initialized_ && !std::isnan(pos_[0]) && !std::isnan(pos_[1]) && 
      !std::isnan(pos_[2]) && !std::isnan(pos_[3])) {
    cmd_[0] = pos_[0];
    cmd_[1] = pos_[1];
    cmd_[2] = pos_[2];
    cmd_[3] = pos_[3];
    target_cmd_[0] = pos_[0];
    target_cmd_[1] = pos_[1];
    target_cmd_[2] = pos_[2];
    target_cmd_[3] = pos_[3];
    first_read_initialized_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "Initialized commands to current positions: [%.3f, %.3f, %.3f, %.3f]",
                 pos_[0], pos_[1], pos_[2], pos_[3]);
  }

  // No effort available from controller in this path; leave eff_ as-is
}

void ShutterPositionInterface::interpolate_to_target_(const rclcpp::Duration& dt)
{
  // Smoothly interpolate current cmd_ toward target_cmd_ using velocity limits
  const double dt_s = dt.seconds();
  
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return;  // Invalid time delta
  }

  for (size_t i = 0; i < 4; ++i)
  {
    const auto& L = limits_[i];
    
    // Apply velocity limiting when moving toward target
    if (L.hard.has_velocity_limits && !std::isnan(cmd_[i]))
    {
      const double max_step = L.hard.max_velocity * dt_s;   // max radians per step
      const double delta = target_cmd_[i] - cmd_[i];
      
      if (std::fabs(delta) > max_step)
      {
        // Move toward target at max velocity
        cmd_[i] += (delta > 0 ? max_step : -max_step);
      }
      else
      {
        // Close enough - set directly to target
        cmd_[i] = target_cmd_[i];
      }
    }
    else
    {
      // No velocity limit - move directly to target
      cmd_[i] = target_cmd_[i];
    }
  }
}

void ShutterPositionInterface::enforce_limits_(const rclcpp::Duration& dt)
{
  // Soft-position clamping (after interpolation)
  for (size_t i = 0; i < 4; ++i)
  {
    // Clamp command by soft-position limits (if present), else hard limits
    const auto& L = limits_[i];
    double min_pos = L.has_soft ? L.soft.min_position :
                      (L.hard.has_position_limits ? L.hard.min_position : -INFINITY);
    double max_pos = L.has_soft ? L.soft.max_position :
                      (L.hard.has_position_limits ? L.hard.max_position :  INFINITY);

    cmd_[i] = clamp(cmd_[i], min_pos, max_pos);
  }
}

void ShutterPositionInterface::write(bool enforce_limits, const rclcpp::Duration& dt)
{
  // First, interpolate current command toward target
  interpolate_to_target_(dt);
  
  // Then apply position limits
  if (enforce_limits)
    enforce_limits_(dt);

  // Send to DynamixelController (kept identical in spirit to ROS1)
  dynamixel_controller_->write(cmd_.data());
}

bool ShutterPositionInterface::jog()
{
  // Set target positions for jog routine
  // keep joint 1 where it is
  target_cmd_[0] = pos_[0];

  // joints 2 & 3: jog to >= -1.54 if below soft min
  target_cmd_[1] = (pos_[1] < -1.54) ? -1.54 : pos_[1];
  target_cmd_[2] = (pos_[2] < -1.54) ? -1.54 : pos_[2];

  // joint 4: if head/shoulder collision risk, jog to 0
  if ((pos_[3] < -0.1) && (pos_[1] < -1.54 || pos_[2] < -1.54))
    target_cmd_[3] = 0.0;
  else
    target_cmd_[3] = pos_[3];
  
  // Also update current cmd to match target for jog (immediate movement)
  cmd_[0] = target_cmd_[0];
  cmd_[1] = target_cmd_[1];
  cmd_[2] = target_cmd_[2];
  cmd_[3] = target_cmd_[3];

  // success if commands are ~at positions (tolerance 0.05)
  return std::equal(cmd_.begin(), cmd_.end(), pos_.begin(),
                    [](double c, double p){ return std::abs(c - p) <= 0.05; });
}
