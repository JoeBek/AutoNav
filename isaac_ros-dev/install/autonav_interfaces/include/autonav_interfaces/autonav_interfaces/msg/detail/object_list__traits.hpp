// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__TRAITS_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autonav_interfaces/msg/detail/object_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'objects'
#include "autonav_interfaces/msg/detail/object__traits.hpp"

namespace autonav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectList & msg,
  std::ostream & out)
{
  out << "{";
  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectList & msg, bool use_flow_style = false)
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
  const autonav_interfaces::msg::ObjectList & msg,
  std::ostream & out, size_t indentation = 0)
{
  autonav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autonav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autonav_interfaces::msg::ObjectList & msg)
{
  return autonav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autonav_interfaces::msg::ObjectList>()
{
  return "autonav_interfaces::msg::ObjectList";
}

template<>
inline const char * name<autonav_interfaces::msg::ObjectList>()
{
  return "autonav_interfaces/msg/ObjectList";
}

template<>
struct has_fixed_size<autonav_interfaces::msg::ObjectList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autonav_interfaces::msg::ObjectList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autonav_interfaces::msg::ObjectList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__TRAITS_HPP_
