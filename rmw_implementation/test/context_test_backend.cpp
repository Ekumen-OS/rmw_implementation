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
rmw_client_t client{};
rmw_service_t service{};
size_t request_limit;
size_t response_limit;
int request_sample;
int response_sample;
size_t publisher_limit;
size_t subscription_limit;
size_t publisher_specific_limit;
size_t subscription_specific_limit;
size_t sample_size = 40;
rmw_ret_t capability_result = RMW_RET_OK;
bool capability_available = true;

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

rmw_ret_t test_service_capability(bool * available)
{
  if (capability_result != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("injected capability query error");
    return capability_result;
  }
  *available = capability_available;
  return RMW_RET_OK;
}
}  // namespace

extern "C"
{
RMW_PUBLIC
void context_test_set_sample_size(size_t size)
{
  sample_size = size;
}

RMW_PUBLIC
void context_test_set_capability(rmw_ret_t result, bool available)
{
  capability_result = result;
  capability_available = available;
}

rmw_ret_t
rmw_region_loan_type_support_available(
  rmw_region_loan_role_t, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t *, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t *, bool * available)
{
  return test_service_capability(available);
}

rmw_ret_t
rmw_region_client_can_loan_messages(const rmw_client_t *, bool * available)
{
  return test_service_capability(available);
}

rmw_ret_t
rmw_region_service_can_loan_messages(const rmw_service_t *, bool * available)
{
  return test_service_capability(available);
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

rmw_client_t *
rmw_create_client_with_context(
  const rmw_node_t *, const rosidl_service_type_support_t *,
  const rosidl_runtime_c__message_context_t * request,
  const rosidl_runtime_c__message_context_t * response,
  const char *, const rmw_qos_profile_t *)
{
  request_limit = request->constraints.max_total_size;
  response_limit = response->constraints.max_total_size;
  return &client;
}

rmw_service_t *
rmw_create_service_with_context(
  const rmw_node_t *, const rosidl_service_type_support_t *,
  const rosidl_runtime_c__message_context_t * request,
  const rosidl_runtime_c__message_context_t * response,
  const char *, const rmw_qos_profile_t *)
{
  request_limit = request->constraints.max_total_size;
  response_limit = response->constraints.max_total_size;
  return &service;
}

rmw_ret_t
rmw_borrow_loaned_request(
  const rmw_client_t *, const rosidl_runtime_c__message_context_t * context, void ** request)
{
  if (!compatible(context, request_limit, 0)) {
    RMW_SET_ERROR_MSG("request baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *request = &request_sample;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_loaned_request(const rmw_client_t *, void ** request, int64_t * sequence_id)
{
  *request = nullptr;
  *sequence_id = 42;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_request_from_client(const rmw_client_t *, void ** request)
{
  *request = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_response(
  const rmw_client_t *, const rosidl_runtime_c__message_context_t * context,
  rmw_service_info_t * header, void ** response, bool * taken)
{
  if (!compatible(context, response_limit, 0)) {
    RMW_SET_ERROR_MSG("response baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *response = &response_sample;
  *taken = true;
  header->request_id.sequence_number = 42;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_response_from_client(const rmw_client_t *, void ** response)
{
  *response = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_request(
  const rmw_service_t *, const rosidl_runtime_c__message_context_t * context,
  rmw_service_info_t * header, void ** request, bool * taken)
{
  if (!compatible(context, request_limit, 0)) {
    RMW_SET_ERROR_MSG("request baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *request = &request_sample;
  *taken = true;
  header->request_id.sequence_number = 42;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_request_from_service(const rmw_service_t *, void ** request)
{
  *request = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_borrow_loaned_response(
  const rmw_service_t *, const rosidl_runtime_c__message_context_t * context, void ** response)
{
  if (!compatible(context, response_limit, 0)) {
    RMW_SET_ERROR_MSG("response baseline exceeded");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *response = &response_sample;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_loaned_response(const rmw_service_t *, const rmw_request_id_t * header, void ** response)
{
  if (header->sequence_number != 42) {
    RMW_SET_ERROR_MSG("wrong request identity");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *response = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_response_from_service(const rmw_service_t *, void ** response)
{
  *response = nullptr;
  return RMW_RET_OK;
}
}  // extern "C"
