// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/MagHeadingStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__MagHeadingStatus __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__MagHeadingStatus __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MagHeadingStatus_
{
  using Type = MagHeadingStatus_<ContainerAllocator>;

  explicit MagHeadingStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MagHeadingStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GOOD =
    0u;
  static constexpr uint8_t OK =
    1u;
  static constexpr uint8_t NOT_GOOD =
    2u;
  static constexpr uint8_t NOT_CALIBRATED =
    3u;
  static constexpr uint8_t MAG_NOT_AVAILABLE =
    4u;

  // pointer types
  using RawPtr =
    zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__MagHeadingStatus
    std::shared_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__MagHeadingStatus
    std::shared_ptr<zed_interfaces::msg::MagHeadingStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MagHeadingStatus_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MagHeadingStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MagHeadingStatus_

// alias to use template instance with default allocator
using MagHeadingStatus =
  zed_interfaces::msg::MagHeadingStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MagHeadingStatus_<ContainerAllocator>::GOOD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MagHeadingStatus_<ContainerAllocator>::OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MagHeadingStatus_<ContainerAllocator>::NOT_GOOD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MagHeadingStatus_<ContainerAllocator>::NOT_CALIBRATED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MagHeadingStatus_<ContainerAllocator>::MAG_NOT_AVAILABLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_HPP_
