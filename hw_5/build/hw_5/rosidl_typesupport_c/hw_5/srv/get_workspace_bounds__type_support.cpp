// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "hw_5/srv/detail/get_workspace_bounds__struct.h"
#include "hw_5/srv/detail/get_workspace_bounds__type_support.h"
#include "hw_5/srv/detail/get_workspace_bounds__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace hw_5
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetWorkspaceBounds_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetWorkspaceBounds_Request_type_support_ids_t;

static const _GetWorkspaceBounds_Request_type_support_ids_t _GetWorkspaceBounds_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetWorkspaceBounds_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetWorkspaceBounds_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetWorkspaceBounds_Request_type_support_symbol_names_t _GetWorkspaceBounds_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hw_5, srv, GetWorkspaceBounds_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hw_5, srv, GetWorkspaceBounds_Request)),
  }
};

typedef struct _GetWorkspaceBounds_Request_type_support_data_t
{
  void * data[2];
} _GetWorkspaceBounds_Request_type_support_data_t;

static _GetWorkspaceBounds_Request_type_support_data_t _GetWorkspaceBounds_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetWorkspaceBounds_Request_message_typesupport_map = {
  2,
  "hw_5",
  &_GetWorkspaceBounds_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetWorkspaceBounds_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetWorkspaceBounds_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetWorkspaceBounds_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetWorkspaceBounds_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &hw_5__srv__GetWorkspaceBounds_Request__get_type_hash,
  &hw_5__srv__GetWorkspaceBounds_Request__get_type_description,
  &hw_5__srv__GetWorkspaceBounds_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace hw_5

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, hw_5, srv, GetWorkspaceBounds_Request)() {
  return &::hw_5::srv::rosidl_typesupport_c::GetWorkspaceBounds_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__struct.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__type_support.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace hw_5
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetWorkspaceBounds_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetWorkspaceBounds_Response_type_support_ids_t;

static const _GetWorkspaceBounds_Response_type_support_ids_t _GetWorkspaceBounds_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetWorkspaceBounds_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetWorkspaceBounds_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetWorkspaceBounds_Response_type_support_symbol_names_t _GetWorkspaceBounds_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hw_5, srv, GetWorkspaceBounds_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hw_5, srv, GetWorkspaceBounds_Response)),
  }
};

typedef struct _GetWorkspaceBounds_Response_type_support_data_t
{
  void * data[2];
} _GetWorkspaceBounds_Response_type_support_data_t;

static _GetWorkspaceBounds_Response_type_support_data_t _GetWorkspaceBounds_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetWorkspaceBounds_Response_message_typesupport_map = {
  2,
  "hw_5",
  &_GetWorkspaceBounds_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetWorkspaceBounds_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetWorkspaceBounds_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetWorkspaceBounds_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetWorkspaceBounds_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &hw_5__srv__GetWorkspaceBounds_Response__get_type_hash,
  &hw_5__srv__GetWorkspaceBounds_Response__get_type_description,
  &hw_5__srv__GetWorkspaceBounds_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace hw_5

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, hw_5, srv, GetWorkspaceBounds_Response)() {
  return &::hw_5::srv::rosidl_typesupport_c::GetWorkspaceBounds_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__struct.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__type_support.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace hw_5
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetWorkspaceBounds_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetWorkspaceBounds_Event_type_support_ids_t;

static const _GetWorkspaceBounds_Event_type_support_ids_t _GetWorkspaceBounds_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetWorkspaceBounds_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetWorkspaceBounds_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetWorkspaceBounds_Event_type_support_symbol_names_t _GetWorkspaceBounds_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hw_5, srv, GetWorkspaceBounds_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hw_5, srv, GetWorkspaceBounds_Event)),
  }
};

typedef struct _GetWorkspaceBounds_Event_type_support_data_t
{
  void * data[2];
} _GetWorkspaceBounds_Event_type_support_data_t;

static _GetWorkspaceBounds_Event_type_support_data_t _GetWorkspaceBounds_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetWorkspaceBounds_Event_message_typesupport_map = {
  2,
  "hw_5",
  &_GetWorkspaceBounds_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetWorkspaceBounds_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetWorkspaceBounds_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetWorkspaceBounds_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetWorkspaceBounds_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &hw_5__srv__GetWorkspaceBounds_Event__get_type_hash,
  &hw_5__srv__GetWorkspaceBounds_Event__get_type_description,
  &hw_5__srv__GetWorkspaceBounds_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace hw_5

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, hw_5, srv, GetWorkspaceBounds_Event)() {
  return &::hw_5::srv::rosidl_typesupport_c::GetWorkspaceBounds_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace hw_5
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _GetWorkspaceBounds_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetWorkspaceBounds_type_support_ids_t;

static const _GetWorkspaceBounds_type_support_ids_t _GetWorkspaceBounds_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetWorkspaceBounds_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetWorkspaceBounds_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetWorkspaceBounds_type_support_symbol_names_t _GetWorkspaceBounds_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hw_5, srv, GetWorkspaceBounds)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hw_5, srv, GetWorkspaceBounds)),
  }
};

typedef struct _GetWorkspaceBounds_type_support_data_t
{
  void * data[2];
} _GetWorkspaceBounds_type_support_data_t;

static _GetWorkspaceBounds_type_support_data_t _GetWorkspaceBounds_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetWorkspaceBounds_service_typesupport_map = {
  2,
  "hw_5",
  &_GetWorkspaceBounds_service_typesupport_ids.typesupport_identifier[0],
  &_GetWorkspaceBounds_service_typesupport_symbol_names.symbol_name[0],
  &_GetWorkspaceBounds_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetWorkspaceBounds_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetWorkspaceBounds_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &GetWorkspaceBounds_Request_message_type_support_handle,
  &GetWorkspaceBounds_Response_message_type_support_handle,
  &GetWorkspaceBounds_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    hw_5,
    srv,
    GetWorkspaceBounds
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    hw_5,
    srv,
    GetWorkspaceBounds
  ),
  &hw_5__srv__GetWorkspaceBounds__get_type_hash,
  &hw_5__srv__GetWorkspaceBounds__get_type_description,
  &hw_5__srv__GetWorkspaceBounds__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace hw_5

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, hw_5, srv, GetWorkspaceBounds)() {
  return &::hw_5::srv::rosidl_typesupport_c::GetWorkspaceBounds_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
