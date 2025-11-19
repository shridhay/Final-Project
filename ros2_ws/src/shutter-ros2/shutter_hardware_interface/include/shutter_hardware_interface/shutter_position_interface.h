#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// ROS 2 joint_limits (replacement for ROS1 joint_limits_interface)
#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_limits_rosparam.hpp>

#include "shutter_hardware_interface/dynamixel_ros_control.h"

/**
 * @brief Lightweight position interface that mirrors the ROS1 class structure
 *        but uses ROS 2 APIs and joint_limits. Not a ros2_control plugin; it’s
 *        a helper you can call from your node/timers or a SystemInterface.
 */
class ShutterPositionInterface
{
public:
  explicit ShutterPositionInterface(
      const rclcpp::Node::SharedPtr& node,
      std::shared_ptr<DynamixelController>& dxl_wb_ptr);

  // Read latest states from DynamixelController into pos_/vel_/eff_
  void read();

  // Write desired positions (cmd_) to DynamixelController
  // If enforce_limits==true, clamps to soft limits and optionally rate limits
  void write(bool enforce_limits = true, const rclcpp::Duration& dt = rclcpp::Duration(0, 100000000)); // 0.1s

  // Same “jog” semantics as your ROS1 version
  bool jog();

  // Accessors
  inline double* cmd_data() { return target_cmd_.data(); }  // Returns target, not current cmd
  inline const double* pos_data() const { return pos_.data(); }
  inline const double* vel_data() const { return vel_.data(); }
  inline const double* eff_data() const { return eff_.data(); }

  // Set target positions (called when new command arrives)
  void setTargetCommand(const std::array<double, 4>& target);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<DynamixelController> dynamixel_controller_;

  // joint arrays (same layout/joint order as ROS1)
  std::array<double, 4> cmd_ {0.0, -1.54, -1.54, 0.0};      // Current interpolated command
  std::array<double, 4> target_cmd_ {0.0, -1.54, -1.54, 0.0}; // Target position from user
  std::array<double, 4> pos_ {NAN, NAN, NAN, NAN};
  std::array<double, 4> vel_ {0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> eff_ {0.0, 0.0, 0.0, 0.0};

  // Limits (mirroring your old hard-coded limits)
  struct JointLimitSet {
    joint_limits::JointLimits hard{};
    joint_limits::SoftJointLimits soft{};
    bool has_soft{false};
  };

  std::array<JointLimitSet, 4> limits_;
  bool first_read_initialized_;

  // Helpers
  void init_limits_hardcoded_();   // uses your original limits
  void enforce_limits_(const rclcpp::Duration& dt);
  void interpolate_to_target_(const rclcpp::Duration& dt);  // Smooth interpolation to target
  static inline double clamp(double v, double lo, double hi)
  { return std::max(lo, std::min(v, hi)); }
};
