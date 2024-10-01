// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/GnssFusionStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zed_interfaces/msg/detail/gnss_fusion_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zed_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GnssFusionStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: gnss_fusion_status
  {
    out << "gnss_fusion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_fusion_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GnssFusionStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gnss_fusion_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_fusion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_fusion_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GnssFusionStatus & msg, bool use_flow_style = false)
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

}  // namespace zed_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zed_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zed_interfaces::msg::GnssFusionStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  zed_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zed_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zed_interfaces::msg::GnssFusionStatus & msg)
{
  return zed_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zed_interfaces::msg::GnssFusionStatus>()
{
  return "zed_interfaces::msg::GnssFusionStatus";
}

template<>
inline const char * name<zed_interfaces::msg::GnssFusionStatus>()
{
  return "zed_interfaces/msg/GnssFusionStatus";
}

template<>
struct has_fixed_size<zed_interfaces::msg::GnssFusionStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zed_interfaces::msg::GnssFusionStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zed_interfaces::msg::GnssFusionStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__TRAITS_HPP_
