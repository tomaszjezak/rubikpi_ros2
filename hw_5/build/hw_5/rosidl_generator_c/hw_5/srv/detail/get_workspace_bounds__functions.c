// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hw_5:srv/GetWorkspaceBounds.idl
// generated code does not contain a copyright notice
#include "hw_5/srv/detail/get_workspace_bounds__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
hw_5__srv__GetWorkspaceBounds_Request__init(hw_5__srv__GetWorkspaceBounds_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Request__fini(hw_5__srv__GetWorkspaceBounds_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
hw_5__srv__GetWorkspaceBounds_Request__are_equal(const hw_5__srv__GetWorkspaceBounds_Request * lhs, const hw_5__srv__GetWorkspaceBounds_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Request__copy(
  const hw_5__srv__GetWorkspaceBounds_Request * input,
  hw_5__srv__GetWorkspaceBounds_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

hw_5__srv__GetWorkspaceBounds_Request *
hw_5__srv__GetWorkspaceBounds_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Request * msg = (hw_5__srv__GetWorkspaceBounds_Request *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hw_5__srv__GetWorkspaceBounds_Request));
  bool success = hw_5__srv__GetWorkspaceBounds_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hw_5__srv__GetWorkspaceBounds_Request__destroy(hw_5__srv__GetWorkspaceBounds_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hw_5__srv__GetWorkspaceBounds_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hw_5__srv__GetWorkspaceBounds_Request__Sequence__init(hw_5__srv__GetWorkspaceBounds_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Request * data = NULL;

  if (size) {
    data = (hw_5__srv__GetWorkspaceBounds_Request *)allocator.zero_allocate(size, sizeof(hw_5__srv__GetWorkspaceBounds_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hw_5__srv__GetWorkspaceBounds_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hw_5__srv__GetWorkspaceBounds_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Request__Sequence__fini(hw_5__srv__GetWorkspaceBounds_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hw_5__srv__GetWorkspaceBounds_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hw_5__srv__GetWorkspaceBounds_Request__Sequence *
hw_5__srv__GetWorkspaceBounds_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Request__Sequence * array = (hw_5__srv__GetWorkspaceBounds_Request__Sequence *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hw_5__srv__GetWorkspaceBounds_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hw_5__srv__GetWorkspaceBounds_Request__Sequence__destroy(hw_5__srv__GetWorkspaceBounds_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hw_5__srv__GetWorkspaceBounds_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hw_5__srv__GetWorkspaceBounds_Request__Sequence__are_equal(const hw_5__srv__GetWorkspaceBounds_Request__Sequence * lhs, const hw_5__srv__GetWorkspaceBounds_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Request__Sequence__copy(
  const hw_5__srv__GetWorkspaceBounds_Request__Sequence * input,
  hw_5__srv__GetWorkspaceBounds_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hw_5__srv__GetWorkspaceBounds_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hw_5__srv__GetWorkspaceBounds_Request * data =
      (hw_5__srv__GetWorkspaceBounds_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hw_5__srv__GetWorkspaceBounds_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hw_5__srv__GetWorkspaceBounds_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
hw_5__srv__GetWorkspaceBounds_Response__init(hw_5__srv__GetWorkspaceBounds_Response * msg)
{
  if (!msg) {
    return false;
  }
  // min_x
  // min_y
  // max_x
  // max_y
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Response__fini(hw_5__srv__GetWorkspaceBounds_Response * msg)
{
  if (!msg) {
    return;
  }
  // min_x
  // min_y
  // max_x
  // max_y
}

bool
hw_5__srv__GetWorkspaceBounds_Response__are_equal(const hw_5__srv__GetWorkspaceBounds_Response * lhs, const hw_5__srv__GetWorkspaceBounds_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // min_x
  if (lhs->min_x != rhs->min_x) {
    return false;
  }
  // min_y
  if (lhs->min_y != rhs->min_y) {
    return false;
  }
  // max_x
  if (lhs->max_x != rhs->max_x) {
    return false;
  }
  // max_y
  if (lhs->max_y != rhs->max_y) {
    return false;
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Response__copy(
  const hw_5__srv__GetWorkspaceBounds_Response * input,
  hw_5__srv__GetWorkspaceBounds_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // min_x
  output->min_x = input->min_x;
  // min_y
  output->min_y = input->min_y;
  // max_x
  output->max_x = input->max_x;
  // max_y
  output->max_y = input->max_y;
  return true;
}

hw_5__srv__GetWorkspaceBounds_Response *
hw_5__srv__GetWorkspaceBounds_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Response * msg = (hw_5__srv__GetWorkspaceBounds_Response *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hw_5__srv__GetWorkspaceBounds_Response));
  bool success = hw_5__srv__GetWorkspaceBounds_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hw_5__srv__GetWorkspaceBounds_Response__destroy(hw_5__srv__GetWorkspaceBounds_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hw_5__srv__GetWorkspaceBounds_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hw_5__srv__GetWorkspaceBounds_Response__Sequence__init(hw_5__srv__GetWorkspaceBounds_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Response * data = NULL;

  if (size) {
    data = (hw_5__srv__GetWorkspaceBounds_Response *)allocator.zero_allocate(size, sizeof(hw_5__srv__GetWorkspaceBounds_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hw_5__srv__GetWorkspaceBounds_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hw_5__srv__GetWorkspaceBounds_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Response__Sequence__fini(hw_5__srv__GetWorkspaceBounds_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hw_5__srv__GetWorkspaceBounds_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hw_5__srv__GetWorkspaceBounds_Response__Sequence *
hw_5__srv__GetWorkspaceBounds_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Response__Sequence * array = (hw_5__srv__GetWorkspaceBounds_Response__Sequence *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hw_5__srv__GetWorkspaceBounds_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hw_5__srv__GetWorkspaceBounds_Response__Sequence__destroy(hw_5__srv__GetWorkspaceBounds_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hw_5__srv__GetWorkspaceBounds_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hw_5__srv__GetWorkspaceBounds_Response__Sequence__are_equal(const hw_5__srv__GetWorkspaceBounds_Response__Sequence * lhs, const hw_5__srv__GetWorkspaceBounds_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Response__Sequence__copy(
  const hw_5__srv__GetWorkspaceBounds_Response__Sequence * input,
  hw_5__srv__GetWorkspaceBounds_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hw_5__srv__GetWorkspaceBounds_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hw_5__srv__GetWorkspaceBounds_Response * data =
      (hw_5__srv__GetWorkspaceBounds_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hw_5__srv__GetWorkspaceBounds_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hw_5__srv__GetWorkspaceBounds_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "hw_5/srv/detail/get_workspace_bounds__functions.h"

bool
hw_5__srv__GetWorkspaceBounds_Event__init(hw_5__srv__GetWorkspaceBounds_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    hw_5__srv__GetWorkspaceBounds_Event__fini(msg);
    return false;
  }
  // request
  if (!hw_5__srv__GetWorkspaceBounds_Request__Sequence__init(&msg->request, 0)) {
    hw_5__srv__GetWorkspaceBounds_Event__fini(msg);
    return false;
  }
  // response
  if (!hw_5__srv__GetWorkspaceBounds_Response__Sequence__init(&msg->response, 0)) {
    hw_5__srv__GetWorkspaceBounds_Event__fini(msg);
    return false;
  }
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Event__fini(hw_5__srv__GetWorkspaceBounds_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  hw_5__srv__GetWorkspaceBounds_Request__Sequence__fini(&msg->request);
  // response
  hw_5__srv__GetWorkspaceBounds_Response__Sequence__fini(&msg->response);
}

bool
hw_5__srv__GetWorkspaceBounds_Event__are_equal(const hw_5__srv__GetWorkspaceBounds_Event * lhs, const hw_5__srv__GetWorkspaceBounds_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!hw_5__srv__GetWorkspaceBounds_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!hw_5__srv__GetWorkspaceBounds_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Event__copy(
  const hw_5__srv__GetWorkspaceBounds_Event * input,
  hw_5__srv__GetWorkspaceBounds_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!hw_5__srv__GetWorkspaceBounds_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!hw_5__srv__GetWorkspaceBounds_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

hw_5__srv__GetWorkspaceBounds_Event *
hw_5__srv__GetWorkspaceBounds_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Event * msg = (hw_5__srv__GetWorkspaceBounds_Event *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hw_5__srv__GetWorkspaceBounds_Event));
  bool success = hw_5__srv__GetWorkspaceBounds_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hw_5__srv__GetWorkspaceBounds_Event__destroy(hw_5__srv__GetWorkspaceBounds_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hw_5__srv__GetWorkspaceBounds_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hw_5__srv__GetWorkspaceBounds_Event__Sequence__init(hw_5__srv__GetWorkspaceBounds_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Event * data = NULL;

  if (size) {
    data = (hw_5__srv__GetWorkspaceBounds_Event *)allocator.zero_allocate(size, sizeof(hw_5__srv__GetWorkspaceBounds_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hw_5__srv__GetWorkspaceBounds_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hw_5__srv__GetWorkspaceBounds_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hw_5__srv__GetWorkspaceBounds_Event__Sequence__fini(hw_5__srv__GetWorkspaceBounds_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hw_5__srv__GetWorkspaceBounds_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hw_5__srv__GetWorkspaceBounds_Event__Sequence *
hw_5__srv__GetWorkspaceBounds_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hw_5__srv__GetWorkspaceBounds_Event__Sequence * array = (hw_5__srv__GetWorkspaceBounds_Event__Sequence *)allocator.allocate(sizeof(hw_5__srv__GetWorkspaceBounds_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hw_5__srv__GetWorkspaceBounds_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hw_5__srv__GetWorkspaceBounds_Event__Sequence__destroy(hw_5__srv__GetWorkspaceBounds_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hw_5__srv__GetWorkspaceBounds_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hw_5__srv__GetWorkspaceBounds_Event__Sequence__are_equal(const hw_5__srv__GetWorkspaceBounds_Event__Sequence * lhs, const hw_5__srv__GetWorkspaceBounds_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hw_5__srv__GetWorkspaceBounds_Event__Sequence__copy(
  const hw_5__srv__GetWorkspaceBounds_Event__Sequence * input,
  hw_5__srv__GetWorkspaceBounds_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hw_5__srv__GetWorkspaceBounds_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hw_5__srv__GetWorkspaceBounds_Event * data =
      (hw_5__srv__GetWorkspaceBounds_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hw_5__srv__GetWorkspaceBounds_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hw_5__srv__GetWorkspaceBounds_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hw_5__srv__GetWorkspaceBounds_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
