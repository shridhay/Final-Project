// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice
#ifndef SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "shutter_face_ros/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "shutter_face_ros/msg/detail/pupils_location__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
bool cdr_serialize_shutter_face_ros__msg__PupilsLocation(
  const shutter_face_ros__msg__PupilsLocation * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
bool cdr_deserialize_shutter_face_ros__msg__PupilsLocation(
  eprosima::fastcdr::Cdr &,
  shutter_face_ros__msg__PupilsLocation * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
size_t get_serialized_size_shutter_face_ros__msg__PupilsLocation(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
size_t max_serialized_size_shutter_face_ros__msg__PupilsLocation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
bool cdr_serialize_key_shutter_face_ros__msg__PupilsLocation(
  const shutter_face_ros__msg__PupilsLocation * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
size_t get_serialized_size_key_shutter_face_ros__msg__PupilsLocation(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
size_t max_serialized_size_key_shutter_face_ros__msg__PupilsLocation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shutter_face_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, shutter_face_ros, msg, PupilsLocation)();

#ifdef __cplusplus
}
#endif

#endif  // SHUTTER_FACE_ROS__MSG__DETAIL__PUPILS_LOCATION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
