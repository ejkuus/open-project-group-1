// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/action/detail/patrol__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `goal`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
turtlebot3_msgs__action__Patrol_Goal__init(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__Vector3__init(&msg->goal)) {
    turtlebot3_msgs__action__Patrol_Goal__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_Goal__fini(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal
  geometry_msgs__msg__Vector3__fini(&msg->goal);
}

bool
turtlebot3_msgs__action__Patrol_Goal__are_equal(const turtlebot3_msgs__action__Patrol_Goal * lhs, const turtlebot3_msgs__action__Patrol_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Goal__copy(
  const turtlebot3_msgs__action__Patrol_Goal * input,
  turtlebot3_msgs__action__Patrol_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_Goal *
turtlebot3_msgs__action__Patrol_Goal__create()
{
  turtlebot3_msgs__action__Patrol_Goal * msg = (turtlebot3_msgs__action__Patrol_Goal *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Goal));
  bool success = turtlebot3_msgs__action__Patrol_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Goal__destroy(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Goal__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Goal__Sequence__init(turtlebot3_msgs__action__Patrol_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Goal * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Goal *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Goal__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_Goal__Sequence__fini(turtlebot3_msgs__action__Patrol_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Goal__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_Goal__Sequence *
turtlebot3_msgs__action__Patrol_Goal__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Goal__Sequence * array = (turtlebot3_msgs__action__Patrol_Goal__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Goal__Sequence__destroy(turtlebot3_msgs__action__Patrol_Goal__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Goal__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_Goal__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_Goal__Sequence * lhs, const turtlebot3_msgs__action__Patrol_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Goal__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_Goal__Sequence * input,
  turtlebot3_msgs__action__Patrol_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_Goal);
    turtlebot3_msgs__action__Patrol_Goal * data =
      (turtlebot3_msgs__action__Patrol_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_Goal__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
#include "rosidl_runtime_c/string_functions.h"

bool
turtlebot3_msgs__action__Patrol_Result__init(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (!msg) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__init(&msg->result)) {
    turtlebot3_msgs__action__Patrol_Result__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_Result__fini(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (!msg) {
    return;
  }
  // result
  rosidl_runtime_c__String__fini(&msg->result);
}

bool
turtlebot3_msgs__action__Patrol_Result__are_equal(const turtlebot3_msgs__action__Patrol_Result * lhs, const turtlebot3_msgs__action__Patrol_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Result__copy(
  const turtlebot3_msgs__action__Patrol_Result * input,
  turtlebot3_msgs__action__Patrol_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_Result *
turtlebot3_msgs__action__Patrol_Result__create()
{
  turtlebot3_msgs__action__Patrol_Result * msg = (turtlebot3_msgs__action__Patrol_Result *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Result));
  bool success = turtlebot3_msgs__action__Patrol_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Result__destroy(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Result__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Result__Sequence__init(turtlebot3_msgs__action__Patrol_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Result * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Result *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Result__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_Result__Sequence__fini(turtlebot3_msgs__action__Patrol_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Result__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_Result__Sequence *
turtlebot3_msgs__action__Patrol_Result__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Result__Sequence * array = (turtlebot3_msgs__action__Patrol_Result__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Result__Sequence__destroy(turtlebot3_msgs__action__Patrol_Result__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Result__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_Result__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_Result__Sequence * lhs, const turtlebot3_msgs__action__Patrol_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Result__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_Result__Sequence * input,
  turtlebot3_msgs__action__Patrol_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_Result);
    turtlebot3_msgs__action__Patrol_Result * data =
      (turtlebot3_msgs__action__Patrol_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_Result__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `state`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
turtlebot3_msgs__action__Patrol_Feedback__init(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    turtlebot3_msgs__action__Patrol_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_Feedback__fini(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
}

bool
turtlebot3_msgs__action__Patrol_Feedback__are_equal(const turtlebot3_msgs__action__Patrol_Feedback * lhs, const turtlebot3_msgs__action__Patrol_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Feedback__copy(
  const turtlebot3_msgs__action__Patrol_Feedback * input,
  turtlebot3_msgs__action__Patrol_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_Feedback *
turtlebot3_msgs__action__Patrol_Feedback__create()
{
  turtlebot3_msgs__action__Patrol_Feedback * msg = (turtlebot3_msgs__action__Patrol_Feedback *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Feedback));
  bool success = turtlebot3_msgs__action__Patrol_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Feedback__destroy(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Feedback__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Feedback__Sequence__init(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Feedback * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Feedback *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_Feedback__Sequence__fini(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_Feedback__Sequence *
turtlebot3_msgs__action__Patrol_Feedback__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Feedback__Sequence * array = (turtlebot3_msgs__action__Patrol_Feedback__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Feedback__Sequence__destroy(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Feedback__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_Feedback__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_Feedback__Sequence * lhs, const turtlebot3_msgs__action__Patrol_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_Feedback__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_Feedback__Sequence * input,
  turtlebot3_msgs__action__Patrol_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_Feedback);
    turtlebot3_msgs__action__Patrol_Feedback * data =
      (turtlebot3_msgs__action__Patrol_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_Feedback__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__init(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!turtlebot3_msgs__action__Patrol_Goal__init(&msg->goal)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  turtlebot3_msgs__action__Patrol_Goal__fini(&msg->goal);
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__are_equal(const turtlebot3_msgs__action__Patrol_SendGoal_Request * lhs, const turtlebot3_msgs__action__Patrol_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!turtlebot3_msgs__action__Patrol_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__copy(
  const turtlebot3_msgs__action__Patrol_SendGoal_Request * input,
  turtlebot3_msgs__action__Patrol_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!turtlebot3_msgs__action__Patrol_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_SendGoal_Request *
turtlebot3_msgs__action__Patrol_SendGoal_Request__create()
{
  turtlebot3_msgs__action__Patrol_SendGoal_Request * msg = (turtlebot3_msgs__action__Patrol_SendGoal_Request *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__init(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_SendGoal_Request * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_SendGoal_Request *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__fini(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence *
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array = (turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * lhs, const turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * input,
  turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request);
    turtlebot3_msgs__action__Patrol_SendGoal_Request * data =
      (turtlebot3_msgs__action__Patrol_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__init(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__are_equal(const turtlebot3_msgs__action__Patrol_SendGoal_Response * lhs, const turtlebot3_msgs__action__Patrol_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__copy(
  const turtlebot3_msgs__action__Patrol_SendGoal_Response * input,
  turtlebot3_msgs__action__Patrol_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_SendGoal_Response *
turtlebot3_msgs__action__Patrol_SendGoal_Response__create()
{
  turtlebot3_msgs__action__Patrol_SendGoal_Response * msg = (turtlebot3_msgs__action__Patrol_SendGoal_Response *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__init(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_SendGoal_Response * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_SendGoal_Response *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__fini(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence *
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array = (turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * lhs, const turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * input,
  turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response);
    turtlebot3_msgs__action__Patrol_SendGoal_Response * data =
      (turtlebot3_msgs__action__Patrol_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__init(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__fini(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__are_equal(const turtlebot3_msgs__action__Patrol_GetResult_Request * lhs, const turtlebot3_msgs__action__Patrol_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__copy(
  const turtlebot3_msgs__action__Patrol_GetResult_Request * input,
  turtlebot3_msgs__action__Patrol_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_GetResult_Request *
turtlebot3_msgs__action__Patrol_GetResult_Request__create()
{
  turtlebot3_msgs__action__Patrol_GetResult_Request * msg = (turtlebot3_msgs__action__Patrol_GetResult_Request *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__destroy(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__init(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_GetResult_Request * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_GetResult_Request *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_GetResult_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__fini(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_GetResult_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence *
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array = (turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__destroy(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * lhs, const turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * input,
  turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request);
    turtlebot3_msgs__action__Patrol_GetResult_Request * data =
      (turtlebot3_msgs__action__Patrol_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_GetResult_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__init(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!turtlebot3_msgs__action__Patrol_Result__init(&msg->result)) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__fini(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  turtlebot3_msgs__action__Patrol_Result__fini(&msg->result);
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__are_equal(const turtlebot3_msgs__action__Patrol_GetResult_Response * lhs, const turtlebot3_msgs__action__Patrol_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!turtlebot3_msgs__action__Patrol_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__copy(
  const turtlebot3_msgs__action__Patrol_GetResult_Response * input,
  turtlebot3_msgs__action__Patrol_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!turtlebot3_msgs__action__Patrol_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_GetResult_Response *
turtlebot3_msgs__action__Patrol_GetResult_Response__create()
{
  turtlebot3_msgs__action__Patrol_GetResult_Response * msg = (turtlebot3_msgs__action__Patrol_GetResult_Response *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__destroy(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__init(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_GetResult_Response * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_GetResult_Response *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_GetResult_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__fini(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_GetResult_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence *
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array = (turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__destroy(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * lhs, const turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * input,
  turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response);
    turtlebot3_msgs__action__Patrol_GetResult_Response * data =
      (turtlebot3_msgs__action__Patrol_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_GetResult_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__init(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!turtlebot3_msgs__action__Patrol_Feedback__init(&msg->feedback)) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  turtlebot3_msgs__action__Patrol_Feedback__fini(&msg->feedback);
}

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__are_equal(const turtlebot3_msgs__action__Patrol_FeedbackMessage * lhs, const turtlebot3_msgs__action__Patrol_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!turtlebot3_msgs__action__Patrol_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__copy(
  const turtlebot3_msgs__action__Patrol_FeedbackMessage * input,
  turtlebot3_msgs__action__Patrol_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!turtlebot3_msgs__action__Patrol_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

turtlebot3_msgs__action__Patrol_FeedbackMessage *
turtlebot3_msgs__action__Patrol_FeedbackMessage__create()
{
  turtlebot3_msgs__action__Patrol_FeedbackMessage * msg = (turtlebot3_msgs__action__Patrol_FeedbackMessage *)malloc(sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
  bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__destroy(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__init(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_FeedbackMessage * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_FeedbackMessage *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__fini(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence *
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array = (turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__destroy(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__are_equal(const turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * lhs, const turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__copy(
  const turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * input,
  turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage);
    turtlebot3_msgs__action__Patrol_FeedbackMessage * data =
      (turtlebot3_msgs__action__Patrol_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__action__Patrol_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__action__Patrol_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
