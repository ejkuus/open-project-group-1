// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:srv/Goal.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__TRAITS_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__TRAITS_HPP_

#include "turtlebot3_msgs/srv/detail/goal__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::srv::Goal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::srv::Goal_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Goal_Request>()
{
  return "turtlebot3_msgs::srv::Goal_Request";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Goal_Request>()
{
  return "turtlebot3_msgs/srv/Goal_Request";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Goal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Goal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::srv::Goal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::srv::Goal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_x: ";
    value_to_yaml(msg.pose_x, out);
    out << "\n";
  }

  // member: pose_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_y: ";
    value_to_yaml(msg.pose_y, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::srv::Goal_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Goal_Response>()
{
  return "turtlebot3_msgs::srv::Goal_Response";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Goal_Response>()
{
  return "turtlebot3_msgs/srv/Goal_Response";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Goal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Goal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::srv::Goal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_msgs::srv::Goal>()
{
  return "turtlebot3_msgs::srv::Goal";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Goal>()
{
  return "turtlebot3_msgs/srv/Goal";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Goal>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_msgs::srv::Goal_Request>::value &&
    has_fixed_size<turtlebot3_msgs::srv::Goal_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Goal>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_msgs::srv::Goal_Request>::value &&
    has_bounded_size<turtlebot3_msgs::srv::Goal_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_msgs::srv::Goal>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_msgs::srv::Goal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_msgs::srv::Goal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__TRAITS_HPP_
