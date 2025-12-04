// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hw_5/srv/get_workspace_bounds.hpp"


#ifndef HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_HPP_
#define HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Request __attribute__((deprecated))
#else
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Request __declspec(deprecated)
#endif

namespace hw_5
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetWorkspaceBounds_Request_
{
  using Type = GetWorkspaceBounds_Request_<ContainerAllocator>;

  explicit GetWorkspaceBounds_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetWorkspaceBounds_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Request
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Request
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetWorkspaceBounds_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetWorkspaceBounds_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetWorkspaceBounds_Request_

// alias to use template instance with default allocator
using GetWorkspaceBounds_Request =
  hw_5::srv::GetWorkspaceBounds_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hw_5


#ifndef _WIN32
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Response __attribute__((deprecated))
#else
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Response __declspec(deprecated)
#endif

namespace hw_5
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetWorkspaceBounds_Response_
{
  using Type = GetWorkspaceBounds_Response_<ContainerAllocator>;

  explicit GetWorkspaceBounds_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_x = 0.0;
      this->min_y = 0.0;
      this->max_x = 0.0;
      this->max_y = 0.0;
    }
  }

  explicit GetWorkspaceBounds_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_x = 0.0;
      this->min_y = 0.0;
      this->max_x = 0.0;
      this->max_y = 0.0;
    }
  }

  // field types and members
  using _min_x_type =
    double;
  _min_x_type min_x;
  using _min_y_type =
    double;
  _min_y_type min_y;
  using _max_x_type =
    double;
  _max_x_type max_x;
  using _max_y_type =
    double;
  _max_y_type max_y;

  // setters for named parameter idiom
  Type & set__min_x(
    const double & _arg)
  {
    this->min_x = _arg;
    return *this;
  }
  Type & set__min_y(
    const double & _arg)
  {
    this->min_y = _arg;
    return *this;
  }
  Type & set__max_x(
    const double & _arg)
  {
    this->max_x = _arg;
    return *this;
  }
  Type & set__max_y(
    const double & _arg)
  {
    this->max_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Response
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Response
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetWorkspaceBounds_Response_ & other) const
  {
    if (this->min_x != other.min_x) {
      return false;
    }
    if (this->min_y != other.min_y) {
      return false;
    }
    if (this->max_x != other.max_x) {
      return false;
    }
    if (this->max_y != other.max_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetWorkspaceBounds_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetWorkspaceBounds_Response_

// alias to use template instance with default allocator
using GetWorkspaceBounds_Response =
  hw_5::srv::GetWorkspaceBounds_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hw_5


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Event __attribute__((deprecated))
#else
# define DEPRECATED__hw_5__srv__GetWorkspaceBounds_Event __declspec(deprecated)
#endif

namespace hw_5
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetWorkspaceBounds_Event_
{
  using Type = GetWorkspaceBounds_Event_<ContainerAllocator>;

  explicit GetWorkspaceBounds_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetWorkspaceBounds_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hw_5::srv::GetWorkspaceBounds_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hw_5::srv::GetWorkspaceBounds_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Event
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hw_5__srv__GetWorkspaceBounds_Event
    std::shared_ptr<hw_5::srv::GetWorkspaceBounds_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetWorkspaceBounds_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetWorkspaceBounds_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetWorkspaceBounds_Event_

// alias to use template instance with default allocator
using GetWorkspaceBounds_Event =
  hw_5::srv::GetWorkspaceBounds_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hw_5

namespace hw_5
{

namespace srv
{

struct GetWorkspaceBounds
{
  using Request = hw_5::srv::GetWorkspaceBounds_Request;
  using Response = hw_5::srv::GetWorkspaceBounds_Response;
  using Event = hw_5::srv::GetWorkspaceBounds_Event;
};

}  // namespace srv

}  // namespace hw_5

#endif  // HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_HPP_
