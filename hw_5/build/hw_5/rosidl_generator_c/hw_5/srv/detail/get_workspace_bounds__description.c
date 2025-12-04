// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice

#include "hw_5/srv/detail/get_workspace_bounds__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hw_5
const rosidl_type_hash_t *
hw_5__srv__GetWorkspaceBounds__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0x7c, 0x35, 0xda, 0x17, 0x41, 0x0d, 0xc5,
      0xab, 0x0e, 0x09, 0xfc, 0xb7, 0xc8, 0x04, 0x11,
      0x4b, 0x98, 0x6e, 0x1a, 0x00, 0x45, 0xbf, 0xc3,
      0xdb, 0xe3, 0x68, 0x1b, 0x50, 0x82, 0x76, 0xad,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_hw_5
const rosidl_type_hash_t *
hw_5__srv__GetWorkspaceBounds_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x31, 0xd1, 0xa8, 0x9a, 0x6c, 0xfd, 0x54, 0x23,
      0x80, 0x3a, 0xec, 0x93, 0x2c, 0x95, 0x7e, 0x91,
      0x9f, 0x6b, 0xc0, 0x35, 0xfb, 0xc5, 0xcd, 0x47,
      0x7b, 0x95, 0x6f, 0x3a, 0x35, 0xeb, 0x14, 0x1e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_hw_5
const rosidl_type_hash_t *
hw_5__srv__GetWorkspaceBounds_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0x0d, 0x0a, 0xd0, 0xe8, 0x5a, 0xc0, 0xc1,
      0xfb, 0x0f, 0x5e, 0x79, 0x19, 0x92, 0xc0, 0x2d,
      0x5c, 0xaa, 0xd3, 0x5f, 0xb2, 0xad, 0x3d, 0x19,
      0x21, 0x7b, 0x51, 0x4d, 0xfa, 0x1a, 0x63, 0xe2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_hw_5
const rosidl_type_hash_t *
hw_5__srv__GetWorkspaceBounds_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1b, 0x16, 0x29, 0x64, 0x68, 0xf2, 0xdc, 0x5e,
      0x75, 0x91, 0x6c, 0x9b, 0xff, 0x8d, 0xe3, 0x07,
      0xb8, 0x49, 0x31, 0xdd, 0x7b, 0x39, 0xab, 0xc7,
      0x70, 0x6d, 0x2d, 0x99, 0xf3, 0xda, 0xc7, 0xc2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char hw_5__srv__GetWorkspaceBounds__TYPE_NAME[] = "hw_5/srv/GetWorkspaceBounds";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char hw_5__srv__GetWorkspaceBounds_Event__TYPE_NAME[] = "hw_5/srv/GetWorkspaceBounds_Event";
static char hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME[] = "hw_5/srv/GetWorkspaceBounds_Request";
static char hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME[] = "hw_5/srv/GetWorkspaceBounds_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char hw_5__srv__GetWorkspaceBounds__FIELD_NAME__request_message[] = "request_message";
static char hw_5__srv__GetWorkspaceBounds__FIELD_NAME__response_message[] = "response_message";
static char hw_5__srv__GetWorkspaceBounds__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field hw_5__srv__GetWorkspaceBounds__FIELDS[] = {
  {
    {hw_5__srv__GetWorkspaceBounds__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {hw_5__srv__GetWorkspaceBounds_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription hw_5__srv__GetWorkspaceBounds__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hw_5__srv__GetWorkspaceBounds__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hw_5__srv__GetWorkspaceBounds__TYPE_NAME, 27, 27},
      {hw_5__srv__GetWorkspaceBounds__FIELDS, 3, 3},
    },
    {hw_5__srv__GetWorkspaceBounds__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = hw_5__srv__GetWorkspaceBounds_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = hw_5__srv__GetWorkspaceBounds_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = hw_5__srv__GetWorkspaceBounds_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char hw_5__srv__GetWorkspaceBounds_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field hw_5__srv__GetWorkspaceBounds_Request__FIELDS[] = {
  {
    {hw_5__srv__GetWorkspaceBounds_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hw_5__srv__GetWorkspaceBounds_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
      {hw_5__srv__GetWorkspaceBounds_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__min_x[] = "min_x";
static char hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__min_y[] = "min_y";
static char hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__max_x[] = "max_x";
static char hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__max_y[] = "max_y";

static rosidl_runtime_c__type_description__Field hw_5__srv__GetWorkspaceBounds_Response__FIELDS[] = {
  {
    {hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__min_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__min_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__max_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Response__FIELD_NAME__max_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hw_5__srv__GetWorkspaceBounds_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
      {hw_5__srv__GetWorkspaceBounds_Response__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__info[] = "info";
static char hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__request[] = "request";
static char hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field hw_5__srv__GetWorkspaceBounds_Event__FIELDS[] = {
  {
    {hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription hw_5__srv__GetWorkspaceBounds_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hw_5__srv__GetWorkspaceBounds_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hw_5__srv__GetWorkspaceBounds_Event__TYPE_NAME, 33, 33},
      {hw_5__srv__GetWorkspaceBounds_Event__FIELDS, 3, 3},
    },
    {hw_5__srv__GetWorkspaceBounds_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = hw_5__srv__GetWorkspaceBounds_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = hw_5__srv__GetWorkspaceBounds_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request (empty - no parameters needed)\n"
  "---\n"
  "# Response\n"
  "float64 min_x\n"
  "float64 min_y\n"
  "float64 max_x\n"
  "float64 max_y";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hw_5__srv__GetWorkspaceBounds__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hw_5__srv__GetWorkspaceBounds__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 112, 112},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
hw_5__srv__GetWorkspaceBounds_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hw_5__srv__GetWorkspaceBounds_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
hw_5__srv__GetWorkspaceBounds_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hw_5__srv__GetWorkspaceBounds_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
hw_5__srv__GetWorkspaceBounds_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hw_5__srv__GetWorkspaceBounds_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hw_5__srv__GetWorkspaceBounds__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hw_5__srv__GetWorkspaceBounds__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *hw_5__srv__GetWorkspaceBounds_Event__get_individual_type_description_source(NULL);
    sources[3] = *hw_5__srv__GetWorkspaceBounds_Request__get_individual_type_description_source(NULL);
    sources[4] = *hw_5__srv__GetWorkspaceBounds_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hw_5__srv__GetWorkspaceBounds_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hw_5__srv__GetWorkspaceBounds_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hw_5__srv__GetWorkspaceBounds_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hw_5__srv__GetWorkspaceBounds_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hw_5__srv__GetWorkspaceBounds_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hw_5__srv__GetWorkspaceBounds_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *hw_5__srv__GetWorkspaceBounds_Request__get_individual_type_description_source(NULL);
    sources[3] = *hw_5__srv__GetWorkspaceBounds_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
