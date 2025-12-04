// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hw_5/srv/get_workspace_bounds.hpp"


#ifndef HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__TRAITS_HPP_
#define HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hw_5/srv/detail/get_workspace_bounds__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hw_5
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetWorkspaceBounds_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetWorkspaceBounds_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetWorkspaceBounds_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hw_5

namespace rosidl_generator_traits
{

[[deprecated("use hw_5::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hw_5::srv::GetWorkspaceBounds_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hw_5::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hw_5::srv::to_yaml() instead")]]
inline std::string to_yaml(const hw_5::srv::GetWorkspaceBounds_Request & msg)
{
  return hw_5::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hw_5::srv::GetWorkspaceBounds_Request>()
{
  return "hw_5::srv::GetWorkspaceBounds_Request";
}

template<>
inline const char * name<hw_5::srv::GetWorkspaceBounds_Request>()
{
  return "hw_5/srv/GetWorkspaceBounds_Request";
}

template<>
struct has_fixed_size<hw_5::srv::GetWorkspaceBounds_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hw_5::srv::GetWorkspaceBounds_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hw_5::srv::GetWorkspaceBounds_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hw_5
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetWorkspaceBounds_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: min_x
  {
    out << "min_x: ";
    rosidl_generator_traits::value_to_yaml(msg.min_x, out);
    out << ", ";
  }

  // member: min_y
  {
    out << "min_y: ";
    rosidl_generator_traits::value_to_yaml(msg.min_y, out);
    out << ", ";
  }

  // member: max_x
  {
    out << "max_x: ";
    rosidl_generator_traits::value_to_yaml(msg.max_x, out);
    out << ", ";
  }

  // member: max_y
  {
    out << "max_y: ";
    rosidl_generator_traits::value_to_yaml(msg.max_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetWorkspaceBounds_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: min_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_x: ";
    rosidl_generator_traits::value_to_yaml(msg.min_x, out);
    out << "\n";
  }

  // member: min_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_y: ";
    rosidl_generator_traits::value_to_yaml(msg.min_y, out);
    out << "\n";
  }

  // member: max_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_x: ";
    rosidl_generator_traits::value_to_yaml(msg.max_x, out);
    out << "\n";
  }

  // member: max_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_y: ";
    rosidl_generator_traits::value_to_yaml(msg.max_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetWorkspaceBounds_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hw_5

namespace rosidl_generator_traits
{

[[deprecated("use hw_5::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hw_5::srv::GetWorkspaceBounds_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hw_5::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hw_5::srv::to_yaml() instead")]]
inline std::string to_yaml(const hw_5::srv::GetWorkspaceBounds_Response & msg)
{
  return hw_5::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hw_5::srv::GetWorkspaceBounds_Response>()
{
  return "hw_5::srv::GetWorkspaceBounds_Response";
}

template<>
inline const char * name<hw_5::srv::GetWorkspaceBounds_Response>()
{
  return "hw_5/srv/GetWorkspaceBounds_Response";
}

template<>
struct has_fixed_size<hw_5::srv::GetWorkspaceBounds_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hw_5::srv::GetWorkspaceBounds_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hw_5::srv::GetWorkspaceBounds_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace hw_5
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetWorkspaceBounds_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const GetWorkspaceBounds_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetWorkspaceBounds_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hw_5

namespace rosidl_generator_traits
{

[[deprecated("use hw_5::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hw_5::srv::GetWorkspaceBounds_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  hw_5::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hw_5::srv::to_yaml() instead")]]
inline std::string to_yaml(const hw_5::srv::GetWorkspaceBounds_Event & msg)
{
  return hw_5::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hw_5::srv::GetWorkspaceBounds_Event>()
{
  return "hw_5::srv::GetWorkspaceBounds_Event";
}

template<>
inline const char * name<hw_5::srv::GetWorkspaceBounds_Event>()
{
  return "hw_5/srv/GetWorkspaceBounds_Event";
}

template<>
struct has_fixed_size<hw_5::srv::GetWorkspaceBounds_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hw_5::srv::GetWorkspaceBounds_Event>
  : std::integral_constant<bool, has_bounded_size<hw_5::srv::GetWorkspaceBounds_Request>::value && has_bounded_size<hw_5::srv::GetWorkspaceBounds_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<hw_5::srv::GetWorkspaceBounds_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hw_5::srv::GetWorkspaceBounds>()
{
  return "hw_5::srv::GetWorkspaceBounds";
}

template<>
inline const char * name<hw_5::srv::GetWorkspaceBounds>()
{
  return "hw_5/srv/GetWorkspaceBounds";
}

template<>
struct has_fixed_size<hw_5::srv::GetWorkspaceBounds>
  : std::integral_constant<
    bool,
    has_fixed_size<hw_5::srv::GetWorkspaceBounds_Request>::value &&
    has_fixed_size<hw_5::srv::GetWorkspaceBounds_Response>::value
  >
{
};

template<>
struct has_bounded_size<hw_5::srv::GetWorkspaceBounds>
  : std::integral_constant<
    bool,
    has_bounded_size<hw_5::srv::GetWorkspaceBounds_Request>::value &&
    has_bounded_size<hw_5::srv::GetWorkspaceBounds_Response>::value
  >
{
};

template<>
struct is_service<hw_5::srv::GetWorkspaceBounds>
  : std::true_type
{
};

template<>
struct is_service_request<hw_5::srv::GetWorkspaceBounds_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hw_5::srv::GetWorkspaceBounds_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__TRAITS_HPP_
