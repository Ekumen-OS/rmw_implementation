// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace
{
rmw_publisher_t publisher{};
rmw_subscription_t subscription{};
size_t publisher_limit;
size_t subscription_limit;
size_t publisher_specific_limit;
size_t subscription_specific_limit;
size_t sample_size = 40;

bool compatible(
  const rosidl_runtime_c__message_context_t * context, size_t baseline, size_t specific_baseline)
{
  if (!context) {
    return true;
  }
  auto requested = context->constraints.max_total_size;
  auto specific = static_cast<const size_t *>(context->constraints.type_specific);
  return (requested == 0 || requested <= baseline) &&
         (!specific || *specific == 0 || *specific <= specific_baseline);
}
}  // namespace

extern "C"
{
RMW_PUBLIC
void context_test_set_sample_size(size_t size)
{
  sample_size = size;
}

rmw_publisher_t *
rmw_create_publisher_with_context(
  const rmw_node_t *, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t * context, const char *,
  const rmw_qos_profile_t *, const rmw_publisher_options_t *)
{
  publisher_limit = context->constraints.max_total_size;
  publisher_specific_limit = *static_cast<const size_t *>(context->constraints.type_specific);
  return &publisher;
}

rmw_subscription_t *
rmw_create_subscription_with_context(
  const rmw_node_t *, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t * context, const char *,
  const rmw_qos_profile_t *, const rmw_subscription_options_t *)
{
  subscription_limit = context->constraints.max_total_size;
  subscription_specific_limit = *static_cast<const size_t *>(context->constraints.type_specific);
  return &subscription;
}

rmw_ret_t
rmw_borrow_loaned_message_with_context(
  const rmw_publisher_t *, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t * context, void ** message)
{
  if (!compatible(context, publisher_limit, publisher_specific_limit)) {
    RMW_SET_ERROR_MSG("publisher baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *message = &publisher_limit;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_message_with_info_and_context(
  const rmw_subscription_t *, const rosidl_runtime_c__message_context_t * context,
  void ** message, bool * taken, rmw_message_info_t * info, rmw_subscription_allocation_t *)
{
  if (!compatible(context, subscription_limit, subscription_specific_limit) ||
    sample_size > subscription_limit ||
    (context && context->constraints.max_total_size &&
    sample_size > context->constraints.max_total_size))
  {
    RMW_SET_ERROR_MSG("subscription baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *message = &subscription_limit;
  *taken = true;
  info->source_timestamp = 42;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_message_with_context(
  const rmw_subscription_t * handle, const rosidl_runtime_c__message_context_t * context,
  void ** message, bool * taken, rmw_subscription_allocation_t * allocation)
{
  rmw_message_info_t info{};
  return rmw_take_loaned_message_with_info_and_context(
    handle, context, message, taken, &info, allocation);
}
}  // extern "C"
