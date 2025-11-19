// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from shutter_face_ros:msg/PupilsLocation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "shutter_face_ros/msg/detail/pupils_location__rosidl_typesupport_introspection_c.h"
#include "shutter_face_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "shutter_face_ros/msg/detail/pupils_location__functions.h"
#include "shutter_face_ros/msg/detail/pupils_location__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `left_eye`
// Member `right_eye`
#include "geometry_msgs/msg/point32.h"
// Member `left_eye`
// Member `right_eye`
#include "geometry_msgs/msg/detail/point32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  shutter_face_ros__msg__PupilsLocation__init(message_memory);
}

void shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_fini_function(void * message_memory)
{
  shutter_face_ros__msg__PupilsLocation__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros__msg__PupilsLocation, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_eye",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros__msg__PupilsLocation, left_eye),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_eye",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shutter_face_ros__msg__PupilsLocation, right_eye),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_members = {
  "shutter_face_ros__msg",  // message namespace
  "PupilsLocation",  // message name
  3,  // number of fields
  sizeof(shutter_face_ros__msg__PupilsLocation),
  false,  // has_any_key_member_
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_member_array,  // message members
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_init_function,  // function to initialize message memory (memory has to be allocated)
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_type_support_handle = {
  0,
  &shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_members,
  get_message_typesupport_handle_function,
  &shutter_face_ros__msg__PupilsLocation__get_type_hash,
  &shutter_face_ros__msg__PupilsLocation__get_type_description,
  &shutter_face_ros__msg__PupilsLocation__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shutter_face_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shutter_face_ros, msg, PupilsLocation)() {
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  if (!shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_type_support_handle.typesupport_identifier) {
    shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &shutter_face_ros__msg__PupilsLocation__rosidl_typesupport_introspection_c__PupilsLocation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
