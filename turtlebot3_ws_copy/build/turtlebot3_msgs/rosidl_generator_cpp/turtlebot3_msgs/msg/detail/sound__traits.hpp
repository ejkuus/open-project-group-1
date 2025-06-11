// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__TRAITS_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__TRAITS_HPP_

#include "turtlebot3_msgs/msg/detail/sound__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::msg::Sound & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::msg::Sound & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::msg::Sound>()
{
  return "turtlebot3_msgs::msg::Sound";
}

template<>
inline const char * name<turtlebot3_msgs::msg::Sound>()
{
  return "turtlebot3_msgs/msg/Sound";
}

template<>
struct has_fixed_size<turtlebot3_msgs::msg::Sound>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::msg::Sound>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::msg::Sound>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__TRAITS_HPP_
