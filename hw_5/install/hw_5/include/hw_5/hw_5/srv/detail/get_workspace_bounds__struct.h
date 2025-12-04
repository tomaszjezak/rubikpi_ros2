// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hw_5/srv/get_workspace_bounds.h"


#ifndef HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_H_
#define HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetWorkspaceBounds in the package hw_5.
typedef struct hw_5__srv__GetWorkspaceBounds_Request
{
  uint8_t structure_needs_at_least_one_member;
} hw_5__srv__GetWorkspaceBounds_Request;

// Struct for a sequence of hw_5__srv__GetWorkspaceBounds_Request.
typedef struct hw_5__srv__GetWorkspaceBounds_Request__Sequence
{
  hw_5__srv__GetWorkspaceBounds_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hw_5__srv__GetWorkspaceBounds_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/GetWorkspaceBounds in the package hw_5.
typedef struct hw_5__srv__GetWorkspaceBounds_Response
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
} hw_5__srv__GetWorkspaceBounds_Response;

// Struct for a sequence of hw_5__srv__GetWorkspaceBounds_Response.
typedef struct hw_5__srv__GetWorkspaceBounds_Response__Sequence
{
  hw_5__srv__GetWorkspaceBounds_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hw_5__srv__GetWorkspaceBounds_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  hw_5__srv__GetWorkspaceBounds_Event__request__MAX_SIZE = 1
};
// response
enum
{
  hw_5__srv__GetWorkspaceBounds_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetWorkspaceBounds in the package hw_5.
typedef struct hw_5__srv__GetWorkspaceBounds_Event
{
  service_msgs__msg__ServiceEventInfo info;
  hw_5__srv__GetWorkspaceBounds_Request__Sequence request;
  hw_5__srv__GetWorkspaceBounds_Response__Sequence response;
} hw_5__srv__GetWorkspaceBounds_Event;

// Struct for a sequence of hw_5__srv__GetWorkspaceBounds_Event.
typedef struct hw_5__srv__GetWorkspaceBounds_Event__Sequence
{
  hw_5__srv__GetWorkspaceBounds_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hw_5__srv__GetWorkspaceBounds_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HW_5__SRV__DETAIL__GET_WORKSPACE_BOUNDS__STRUCT_H_
