// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonav_interfaces/msg/detail/object_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonav_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectList_objects
{
public:
  Init_ObjectList_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonav_interfaces::msg::ObjectList objects(::autonav_interfaces::msg::ObjectList::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonav_interfaces::msg::ObjectList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonav_interfaces::msg::ObjectList>()
{
  return autonav_interfaces::msg::builder::Init_ObjectList_objects();
}

}  // namespace autonav_interfaces

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_
