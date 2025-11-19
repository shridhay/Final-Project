// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shutter_face_ros/msg/pupils_location.hpp"


#ifndef SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_HPP_
#define SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'left_eye'
// Member 'right_eye'
#include "geometry_msgs/msg/detail/point32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__shutter_face_ros__msg__PupilsLocation __attribute__((deprecated))
#else
# define DEPRECATED__shutter_face_ros__msg__PupilsLocation __declspec(deprecated)
#endif

namespace shutter_face_ros
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PupilsLocation_
{
  using Type = PupilsLocation_<ContainerAllocator>;

  explicit PupilsLocation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left_eye(_init),
    right_eye(_init)
  {
    (void)_init;
  }

  explicit PupilsLocation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left_eye(_alloc, _init),
    right_eye(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_eye_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _left_eye_type left_eye;
  using _right_eye_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _right_eye_type right_eye;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_eye(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->left_eye = _arg;
    return *this;
  }
  Type & set__right_eye(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->right_eye = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> *;
  using ConstRawPtr =
    const shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shutter_face_ros__msg__PupilsLocation
    std::shared_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shutter_face_ros__msg__PupilsLocation
    std::shared_ptr<shutter_face_ros::msg::PupilsLocation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PupilsLocation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_eye != other.left_eye) {
      return false;
    }
    if (this->right_eye != other.right_eye) {
      return false;
    }
    return true;
  }
  bool operator!=(const PupilsLocation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PupilsLocation_

// alias to use template instance with default allocator
using PupilsLocation =
  shutter_face_ros::msg::PupilsLocation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shutter_face_ros

#endif  // SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_HPP_
