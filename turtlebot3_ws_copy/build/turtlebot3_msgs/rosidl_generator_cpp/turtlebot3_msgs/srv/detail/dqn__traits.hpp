// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_

#include "turtlebot3_msgs/srv/detail/dqn__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::srv::Dqn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "init: ";
    value_to_yaml(msg.init, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::srv::Dqn_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn_Request>()
{
  return "turtlebot3_msgs::srv::Dqn_Request";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn_Request>()
{
  return "turtlebot3_msgs/srv/Dqn_Request";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::srv::Dqn_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turtlebot3_msgs::srv::Dqn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state.size() == 0) {
      out << "state: []\n";
    } else {
      out << "state:\n";
      for (auto item : msg.state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reward: ";
    value_to_yaml(msg.reward, out);
    out << "\n";
  }

  // member: done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "done: ";
    value_to_yaml(msg.done, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turtlebot3_msgs::srv::Dqn_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn_Response>()
{
  return "turtlebot3_msgs::srv::Dqn_Response";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn_Response>()
{
  return "turtlebot3_msgs/srv/Dqn_Response";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_msgs::srv::Dqn_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn>()
{
  return "turtlebot3_msgs::srv::Dqn";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn>()
{
  return "turtlebot3_msgs/srv/Dqn";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_msgs::srv::Dqn_Request>::value &&
    has_fixed_size<turtlebot3_msgs::srv::Dqn_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_msgs::srv::Dqn_Request>::value &&
    has_bounded_size<turtlebot3_msgs::srv::Dqn_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_msgs::srv::Dqn>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_msgs::srv::Dqn_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_msgs::srv::Dqn_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_
