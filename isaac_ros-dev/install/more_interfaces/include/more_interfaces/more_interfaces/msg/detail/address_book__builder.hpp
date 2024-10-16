// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from more_interfaces:msg/AddressBook.idl
// generated code does not contain a copyright notice

#ifndef MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_
#define MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "more_interfaces/msg/detail/address_book__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace more_interfaces
{

namespace msg
{

namespace builder
{

class Init_AddressBook_phone_type
{
public:
  explicit Init_AddressBook_phone_type(::more_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  ::more_interfaces::msg::AddressBook phone_type(::more_interfaces::msg::AddressBook::_phone_type_type arg)
  {
    msg_.phone_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::more_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_phone_number
{
public:
  explicit Init_AddressBook_phone_number(::more_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  Init_AddressBook_phone_type phone_number(::more_interfaces::msg::AddressBook::_phone_number_type arg)
  {
    msg_.phone_number = std::move(arg);
    return Init_AddressBook_phone_type(msg_);
  }

private:
  ::more_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_last_name
{
public:
  explicit Init_AddressBook_last_name(::more_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  Init_AddressBook_phone_number last_name(::more_interfaces::msg::AddressBook::_last_name_type arg)
  {
    msg_.last_name = std::move(arg);
    return Init_AddressBook_phone_number(msg_);
  }

private:
  ::more_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_first_name
{
public:
  Init_AddressBook_first_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddressBook_last_name first_name(::more_interfaces::msg::AddressBook::_first_name_type arg)
  {
    msg_.first_name = std::move(arg);
    return Init_AddressBook_last_name(msg_);
  }

private:
  ::more_interfaces::msg::AddressBook msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::more_interfaces::msg::AddressBook>()
{
  return more_interfaces::msg::builder::Init_AddressBook_first_name();
}

}  // namespace more_interfaces

#endif  // MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_
