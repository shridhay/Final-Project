// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shutter_face_ros/msg/pupils_location.hpp"


#ifndef SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__TRAITS_HPP_
#define SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shutter_face_ros/msg/detail/pupils_location__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'left_eye'
// Member 'right_eye'
#include "geometry_msgs/msg/detail/point32__traits.hpp"

namespace shutter_face_ros
{

namespace msg
{

inline void to_flow_style_yaml(
  const PupilsLocation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: left_eye
  {
    out << "left_eye: ";
    to_flow_style_yaml(msg.left_eye, out);
    out << ", ";
  }

  // member: right_eye
  {
    out << "right_eye: ";
    to_flow_style_yaml(msg.right_eye, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PupilsLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: left_eye
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_eye:\n";
    to_block_style_yaml(msg.left_eye, out, indentation + 2);
  }

  // member: right_eye
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_eye:\n";
    to_block_style_yaml(msg.right_eye, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PupilsLocation & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace shutter_face_ros

namespace rosidl_generator_traits
{

[[deprecated("use shutter_face_ros::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shutter_face_ros::msg::PupilsLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  shutter_face_ros::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shutter_face_ros::msg::to_yaml() instead")]]
inline std::string to_yaml(const shutter_face_ros::msg::PupilsLocation & msg)
{
  return shutter_face_ros::msg::to_yaml(msg);
}

template<>
inline const char * data_type<shutter_face_ros::msg::PupilsLocation>()
{
  return "shutter_face_ros::msg::PupilsLocation";
}

template<>
inline const char * name<shutter_face_ros::msg::PupilsLocation>()
{
  return "shutter_face_ros/msg/PupilsLocation";
}

template<>
struct has_fixed_size<shutter_face_ros::msg::PupilsLocation>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point32>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<shutter_face_ros::msg::PupilsLocation>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point32>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<shutter_face_ros::msg::PupilsLocation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__TRAITS_HPP_
