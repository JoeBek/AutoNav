// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'objects'
#include "autonav_interfaces/msg/detail/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autonav_interfaces__msg__ObjectList __attribute__((deprecated))
#else
# define DEPRECATED__autonav_interfaces__msg__ObjectList __declspec(deprecated)
#endif

namespace autonav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectList_
{
  using Type = ObjectList_<ContainerAllocator>;

  explicit ObjectList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ObjectList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<autonav_interfaces::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonav_interfaces::msg::Object_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<autonav_interfaces::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autonav_interfaces::msg::Object_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autonav_interfaces::msg::ObjectList_<ContainerAllocator> *;
  using ConstRawPtr =
    const autonav_interfaces::msg::ObjectList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autonav_interfaces::msg::ObjectList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autonav_interfaces::msg::ObjectList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autonav_interfaces__msg__ObjectList
    std::shared_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autonav_interfaces__msg__ObjectList
    std::shared_ptr<autonav_interfaces::msg::ObjectList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectList_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectList_

// alias to use template instance with default allocator
using ObjectList =
  autonav_interfaces::msg::ObjectList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autonav_interfaces

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_
