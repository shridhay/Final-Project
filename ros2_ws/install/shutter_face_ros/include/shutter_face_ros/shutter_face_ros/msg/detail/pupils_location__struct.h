// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shutter_face_ros/msg/pupils_location.h"


#ifndef SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_H_
#define SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'left_eye'
// Member 'right_eye'
#include "geometry_msgs/msg/detail/point32__struct.h"

/// Struct defined in msg/PupilsLocation in the package shutter_face_ros.
typedef struct shutter_face_ros__msg__PupilsLocation
{
  /// stamp
  std_msgs__msg__Header header;
  /// 2D pupil location for the left eye -- ignore z value
  geometry_msgs__msg__Point32 left_eye;
  /// 2D pupil location for the right eye -- ignore z value
  geometry_msgs__msg__Point32 right_eye;
} shutter_face_ros__msg__PupilsLocation;

// Struct for a sequence of shutter_face_ros__msg__PupilsLocation.
typedef struct shutter_face_ros__msg__PupilsLocation__Sequence
{
  shutter_face_ros__msg__PupilsLocation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shutter_face_ros__msg__PupilsLocation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__STRUCT_H_
