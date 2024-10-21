// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonav_interfaces/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonav_interfaces
{

namespace msg
{

namespace builder
{

class Init_Object_z
{
public:
  explicit Init_Object_z(::autonav_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  ::autonav_interfaces::msg::Object z(::autonav_interfaces::msg::Object::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonav_interfaces::msg::Object msg_;
};

class Init_Object_x
{
public:
  Init_Object_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_z x(::autonav_interfaces::msg::Object::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Object_z(msg_);
  }

private:
  ::autonav_interfaces::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonav_interfaces::msg::Object>()
{
  return autonav_interfaces::msg::builder::Init_Object_x();
}

}  // namespace autonav_interfaces

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
