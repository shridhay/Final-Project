// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shutter_face_ros/msg/pupils_location.hpp"


#ifndef SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__BUILDER_HPP_
#define SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shutter_face_ros/msg/detail/pupils_location__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shutter_face_ros
{

namespace msg
{

namespace builder
{

class Init_PupilsLocation_right_eye
{
public:
  explicit Init_PupilsLocation_right_eye(::shutter_face_ros::msg::PupilsLocation & msg)
  : msg_(msg)
  {}
  ::shutter_face_ros::msg::PupilsLocation right_eye(::shutter_face_ros::msg::PupilsLocation::_right_eye_type arg)
  {
    msg_.right_eye = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shutter_face_ros::msg::PupilsLocation msg_;
};

class Init_PupilsLocation_left_eye
{
public:
  explicit Init_PupilsLocation_left_eye(::shutter_face_ros::msg::PupilsLocation & msg)
  : msg_(msg)
  {}
  Init_PupilsLocation_right_eye left_eye(::shutter_face_ros::msg::PupilsLocation::_left_eye_type arg)
  {
    msg_.left_eye = std::move(arg);
    return Init_PupilsLocation_right_eye(msg_);
  }

private:
  ::shutter_face_ros::msg::PupilsLocation msg_;
};

class Init_PupilsLocation_header
{
public:
  Init_PupilsLocation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PupilsLocation_left_eye header(::shutter_face_ros::msg::PupilsLocation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PupilsLocation_left_eye(msg_);
  }

private:
  ::shutter_face_ros::msg::PupilsLocation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shutter_face_ros::msg::PupilsLocation>()
{
  return shutter_face_ros::msg::builder::Init_PupilsLocation_header();
}

}  // namespace shutter_face_ros

#endif  // SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__BUILDER_HPP_
