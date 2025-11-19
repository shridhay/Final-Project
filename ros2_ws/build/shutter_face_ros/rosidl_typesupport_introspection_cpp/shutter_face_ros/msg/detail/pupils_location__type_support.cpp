// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "shutter_face_ros/msg/detail/pupils_location__functions.h"
#include "shutter_face_ros/msg/detail/pupils_location__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace shutter_face_ros
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PupilsLocation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) shutter_face_ros::msg::PupilsLocation(_init);
}

void PupilsLocation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<shutter_face_ros::msg::PupilsLocation *>(message_memory);
  typed_message->~PupilsLocation();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PupilsLocation_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros::msg::PupilsLocation, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_eye",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point32>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros::msg::PupilsLocation, left_eye),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_eye",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point32>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros::msg::PupilsLocation, right_eye),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PupilsLocation_message_members = {
  "shutter_face_ros::msg",  // message namespace
  "PupilsLocation",  // message name
  3,  // number of fields
  sizeof(shutter_face_ros::msg::PupilsLocation),
  false,  // has_any_key_member_
  PupilsLocation_message_member_array,  // message members
  PupilsLocation_init_function,  // function to initialize message memory (memory has to be allocated)
  PupilsLocation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PupilsLocation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PupilsLocation_message_members,
  get_message_typesupport_handle_function,
  &shutter_face_ros__msg__PupilsLocation__get_type_hash,
  &shutter_face_ros__msg__PupilsLocation__get_type_description,
  &shutter_face_ros__msg__PupilsLocation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace shutter_face_ros


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<shutter_face_ros::msg::PupilsLocation>()
{
  return &::shutter_face_ros::msg::rosidl_typesupport_introspection_cpp::PupilsLocation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, shutter_face_ros, msg, PupilsLocation)() {
  return &::shutter_face_ros::msg::rosidl_typesupport_introspection_cpp::PupilsLocation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
