// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from rslidar_msg:msg/RslidarPacket.idl
// generated code does not contain a copyright notice
#ifndef RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rslidar_msg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rslidar_msg/msg/detail/rslidar_packet__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
bool cdr_serialize_rslidar_msg__msg__RslidarPacket(
  const rslidar_msg__msg__RslidarPacket * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
bool cdr_deserialize_rslidar_msg__msg__RslidarPacket(
  eprosima::fastcdr::Cdr &,
  rslidar_msg__msg__RslidarPacket * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
size_t get_serialized_size_rslidar_msg__msg__RslidarPacket(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
size_t max_serialized_size_rslidar_msg__msg__RslidarPacket(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
bool cdr_serialize_key_rslidar_msg__msg__RslidarPacket(
  const rslidar_msg__msg__RslidarPacket * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
size_t get_serialized_size_key_rslidar_msg__msg__RslidarPacket(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
size_t max_serialized_size_key_rslidar_msg__msg__RslidarPacket(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rslidar_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rslidar_msg, msg, RslidarPacket)();

#ifdef __cplusplus
}
#endif

#endif  // RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
