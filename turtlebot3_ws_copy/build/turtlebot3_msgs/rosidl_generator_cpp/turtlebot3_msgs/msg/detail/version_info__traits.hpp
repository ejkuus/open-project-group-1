// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:msg/VersionInfo.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__TRAITS_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__TRAITS_HPP_

#include "turtlebot3_msgs/msg/detail/version_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::msg::VersionInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: hardware
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware: ";
    value_to_yaml(msg.hardware, out);
    out << "\n";
  }

  // member: firmware
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware: ";
    value_to_yaml(msg.firmware, out);
    out << "\n";
  }

  // member: software
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "software: ";
    value_to_yaml(msg.software, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::msg::VersionInfo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::msg::VersionInfo>()
{
  return "turtlebot3_msgs::msg::VersionInfo";
}

template<>
inline const char * name<turtlebot3_msgs::msg::VersionInfo>()
{
  return "turtlebot3_msgs/msg/VersionInfo";
}

template<>
struct has_fixed_size<turtlebot3_msgs::msg::VersionInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_msgs::msg::VersionInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_msgs::msg::VersionInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__TRAITS_HPP_
