// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from more_interfaces:msg/AddressBook.idl
// generated code does not contain a copyright notice

#ifndef MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__STRUCT_HPP_
#define MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__more_interfaces__msg__AddressBook __attribute__((deprecated))
#else
# define DEPRECATED__more_interfaces__msg__AddressBook __declspec(deprecated)
#endif

namespace more_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AddressBook_
{
  using Type = AddressBook_<ContainerAllocator>;

  explicit AddressBook_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_name = "";
      this->last_name = "";
      this->phone_number = "";
      this->phone_type = 0;
    }
  }

  explicit AddressBook_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : first_name(_alloc),
    last_name(_alloc),
    phone_number(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_name = "";
      this->last_name = "";
      this->phone_number = "";
      this->phone_type = 0;
    }
  }

  // field types and members
  using _first_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _first_name_type first_name;
  using _last_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_name_type last_name;
  using _phone_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phone_number_type phone_number;
  using _phone_type_type =
    uint8_t;
  _phone_type_type phone_type;

  // setters for named parameter idiom
  Type & set__first_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->first_name = _arg;
    return *this;
  }
  Type & set__last_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_name = _arg;
    return *this;
  }
  Type & set__phone_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phone_number = _arg;
    return *this;
  }
  Type & set__phone_type(
    const uint8_t & _arg)
  {
    this->phone_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t PHONE_TYPE_HOME =
    0u;
  static constexpr uint8_t PHONE_TYPE_WORK =
    1u;
  static constexpr uint8_t PHONE_TYPE_MOBILE =
    2u;

  // pointer types
  using RawPtr =
    more_interfaces::msg::AddressBook_<ContainerAllocator> *;
  using ConstRawPtr =
    const more_interfaces::msg::AddressBook_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      more_interfaces::msg::AddressBook_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      more_interfaces::msg::AddressBook_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__more_interfaces__msg__AddressBook
    std::shared_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__more_interfaces__msg__AddressBook
    std::shared_ptr<more_interfaces::msg::AddressBook_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AddressBook_ & other) const
  {
    if (this->first_name != other.first_name) {
      return false;
    }
    if (this->last_name != other.last_name) {
      return false;
    }
    if (this->phone_number != other.phone_number) {
      return false;
    }
    if (this->phone_type != other.phone_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const AddressBook_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AddressBook_

// alias to use template instance with default allocator
using AddressBook =
  more_interfaces::msg::AddressBook_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AddressBook_<ContainerAllocator>::PHONE_TYPE_HOME;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AddressBook_<ContainerAllocator>::PHONE_TYPE_WORK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AddressBook_<ContainerAllocator>::PHONE_TYPE_MOBILE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace more_interfaces

#endif  // MORE_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__STRUCT_HPP_
