// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hw_5/srv/get_workspace_bounds.hpp"


#ifndef HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__BUILDER_HPP_
#define HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hw_5/srv/detail/get_workspace_bounds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hw_5
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hw_5::srv::GetWorkspaceBounds_Request>()
{
  return ::hw_5::srv::GetWorkspaceBounds_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hw_5


namespace hw_5
{

namespace srv
{

namespace builder
{

class Init_GetWorkspaceBounds_Response_max_y
{
public:
  explicit Init_GetWorkspaceBounds_Response_max_y(::hw_5::srv::GetWorkspaceBounds_Response & msg)
  : msg_(msg)
  {}
  ::hw_5::srv::GetWorkspaceBounds_Response max_y(::hw_5::srv::GetWorkspaceBounds_Response::_max_y_type arg)
  {
    msg_.max_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Response msg_;
};

class Init_GetWorkspaceBounds_Response_max_x
{
public:
  explicit Init_GetWorkspaceBounds_Response_max_x(::hw_5::srv::GetWorkspaceBounds_Response & msg)
  : msg_(msg)
  {}
  Init_GetWorkspaceBounds_Response_max_y max_x(::hw_5::srv::GetWorkspaceBounds_Response::_max_x_type arg)
  {
    msg_.max_x = std::move(arg);
    return Init_GetWorkspaceBounds_Response_max_y(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Response msg_;
};

class Init_GetWorkspaceBounds_Response_min_y
{
public:
  explicit Init_GetWorkspaceBounds_Response_min_y(::hw_5::srv::GetWorkspaceBounds_Response & msg)
  : msg_(msg)
  {}
  Init_GetWorkspaceBounds_Response_max_x min_y(::hw_5::srv::GetWorkspaceBounds_Response::_min_y_type arg)
  {
    msg_.min_y = std::move(arg);
    return Init_GetWorkspaceBounds_Response_max_x(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Response msg_;
};

class Init_GetWorkspaceBounds_Response_min_x
{
public:
  Init_GetWorkspaceBounds_Response_min_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetWorkspaceBounds_Response_min_y min_x(::hw_5::srv::GetWorkspaceBounds_Response::_min_x_type arg)
  {
    msg_.min_x = std::move(arg);
    return Init_GetWorkspaceBounds_Response_min_y(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hw_5::srv::GetWorkspaceBounds_Response>()
{
  return hw_5::srv::builder::Init_GetWorkspaceBounds_Response_min_x();
}

}  // namespace hw_5


namespace hw_5
{

namespace srv
{

namespace builder
{

class Init_GetWorkspaceBounds_Event_response
{
public:
  explicit Init_GetWorkspaceBounds_Event_response(::hw_5::srv::GetWorkspaceBounds_Event & msg)
  : msg_(msg)
  {}
  ::hw_5::srv::GetWorkspaceBounds_Event response(::hw_5::srv::GetWorkspaceBounds_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Event msg_;
};

class Init_GetWorkspaceBounds_Event_request
{
public:
  explicit Init_GetWorkspaceBounds_Event_request(::hw_5::srv::GetWorkspaceBounds_Event & msg)
  : msg_(msg)
  {}
  Init_GetWorkspaceBounds_Event_response request(::hw_5::srv::GetWorkspaceBounds_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetWorkspaceBounds_Event_response(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Event msg_;
};

class Init_GetWorkspaceBounds_Event_info
{
public:
  Init_GetWorkspaceBounds_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetWorkspaceBounds_Event_request info(::hw_5::srv::GetWorkspaceBounds_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetWorkspaceBounds_Event_request(msg_);
  }

private:
  ::hw_5::srv::GetWorkspaceBounds_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hw_5::srv::GetWorkspaceBounds_Event>()
{
  return hw_5::srv::builder::Init_GetWorkspaceBounds_Event_info();
}

}  // namespace hw_5

#endif  // HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__BUILDER_HPP_
