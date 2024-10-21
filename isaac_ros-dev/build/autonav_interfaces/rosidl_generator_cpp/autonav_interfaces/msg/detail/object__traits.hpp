// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonav_interfaces/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autonav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Object & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Object & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Object & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autonav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autonav_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autonav_interfaces::msg::Object & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonav_interfaces::msg::Object & msg)
{
  return autonav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonav_interfaces::msg::Object>()
{
  return "autonav_interfaces::msg::Object";
}

template<>
inline const char * name<autonav_interfaces::msg::Object>()
{
  return "autonav_interfaces/msg/Object";
}

template<>
struct has_fixed_size<autonav_interfaces::msg::Object>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autonav_interfaces::msg::Object>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autonav_interfaces::msg::Object>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
