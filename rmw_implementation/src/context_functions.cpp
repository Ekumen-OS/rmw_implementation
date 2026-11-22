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

#include <memory>
#include <stdexcept>

#include "rcpputils/shared_library.hpp"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#ifndef RMW_CONTEXT_FIXED_IMPLEMENTATION
#include "functions.hpp"
#endif

#define STRINGIFY_CONTEXT_(s) #s
#define STRINGIFY_CONTEXT(s) STRINGIFY_CONTEXT_(s)

static void *
get_optional_symbol(const char * symbol_name, bool & failed)
{
  failed = false;
  try {
#ifdef RMW_CONTEXT_FIXED_IMPLEMENTATION
    static auto library = std::make_shared<rcpputils::SharedLibrary>(
      rcpputils::get_platform_library_name(STRINGIFY_CONTEXT(RMW_CONTEXT_FIXED_IMPLEMENTATION)));
#else
    auto library = get_library();
#endif
    if (!library) {
      failed = true;
      return nullptr;
    }
    if (!library->has_symbol(symbol_name)) {
      return nullptr;
    }
    return library->get_symbol(symbol_name);
  } catch (const std::exception & e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to get symbol '%s' due to %s", symbol_name, e.what());
    failed = true;
    return nullptr;
  }
}

template<typename Function, typename ... Args>
static rmw_ret_t
call_optional_service_loan(const char * name, Args ... args)
{
  bool failed;
  auto symbol = get_optional_symbol(name, failed);
  if (failed) {
    return RMW_RET_ERROR;
  }
  if (!symbol) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("middleware does not support %s", name);
    return RMW_RET_UNSUPPORTED;
  }
  return reinterpret_cast<Function>(symbol)(args ...);
}

extern "C"
{
rmw_publisher_t *
rmw_create_publisher_with_context(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__message_context_t * context,
  const char * topic_name,
  const rmw_qos_profile_t * qos,
  const rmw_publisher_options_t * options)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_create_publisher_with_context", failed);
  if (failed) {
    return nullptr;
  }
  if (symbol) {
    auto function = reinterpret_cast<decltype(& rmw_create_publisher_with_context)>(symbol);
    return function(node, type_support, context, topic_name, qos, options);
  }
  if (!context) {
    return rmw_create_publisher(node, type_support, topic_name, qos, options);
  }
  RMW_SET_ERROR_MSG("middleware does not support publisher message contexts");
  return nullptr;
}

rmw_ret_t
rmw_borrow_loaned_message_with_context(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__message_context_t * context,
  void ** message)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_borrow_loaned_message_with_context", failed);
  if (failed) {
    return RMW_RET_ERROR;
  }
  if (symbol) {
    auto function = reinterpret_cast<decltype(& rmw_borrow_loaned_message_with_context)>(symbol);
    return function(publisher, type_support, context, message);
  }
  if (!context) {
    return rmw_borrow_loaned_message(publisher, type_support, message);
  }
  RMW_SET_ERROR_MSG("middleware does not support loan message contexts");
  return RMW_RET_UNSUPPORTED;
}

rmw_subscription_t *
rmw_create_subscription_with_context(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__message_context_t * context,
  const char * topic_name,
  const rmw_qos_profile_t * qos,
  const rmw_subscription_options_t * options)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_create_subscription_with_context", failed);
  if (failed) {
    return nullptr;
  }
  if (symbol) {
    auto function = reinterpret_cast<decltype(& rmw_create_subscription_with_context)>(symbol);
    return function(node, type_support, context, topic_name, qos, options);
  }
  if (!context) {
    return rmw_create_subscription(node, type_support, topic_name, qos, options);
  }
  RMW_SET_ERROR_MSG("middleware does not support subscription message contexts");
  return nullptr;
}

rmw_ret_t
rmw_take_loaned_message_with_context(
  const rmw_subscription_t * subscription,
  const rosidl_runtime_c__message_context_t * context,
  void ** message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_take_loaned_message_with_context", failed);
  if (failed) {
    return RMW_RET_ERROR;
  }
  if (symbol) {
    auto function = reinterpret_cast<decltype(& rmw_take_loaned_message_with_context)>(symbol);
    return function(subscription, context, message, taken, allocation);
  }
  if (!context) {
    return rmw_take_loaned_message(subscription, message, taken, allocation);
  }
  RMW_SET_ERROR_MSG("middleware does not support loan message contexts");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message_with_info_and_context(
  const rmw_subscription_t * subscription,
  const rosidl_runtime_c__message_context_t * context,
  void ** message,
  bool * taken,
  rmw_message_info_t * info,
  rmw_subscription_allocation_t * allocation)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_take_loaned_message_with_info_and_context", failed);
  if (failed) {
    return RMW_RET_ERROR;
  }
  if (symbol) {
    auto function = reinterpret_cast<decltype(& rmw_take_loaned_message_with_info_and_context)>(
      symbol);
    return function(subscription, context, message, taken, info, allocation);
  }
  if (!context) {
    return rmw_take_loaned_message_with_info(subscription, message, taken, info, allocation);
  }
  RMW_SET_ERROR_MSG("middleware does not support loan message contexts");
  return RMW_RET_UNSUPPORTED;
}

rmw_client_t *
rmw_create_client_with_context(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const rosidl_runtime_c__message_context_t * request_context,
  const rosidl_runtime_c__message_context_t * response_context,
  const char * service_name,
  const rmw_qos_profile_t * qos)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_create_client_with_context", failed);
  if (failed) {
    return nullptr;
  }
  if (!symbol) {
    RMW_SET_ERROR_MSG("middleware does not support client message contexts");
    return nullptr;
  }
  return reinterpret_cast<decltype(& rmw_create_client_with_context)>(symbol)(
    node, type_support, request_context, response_context, service_name, qos);
}

rmw_service_t *
rmw_create_service_with_context(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const rosidl_runtime_c__message_context_t * request_context,
  const rosidl_runtime_c__message_context_t * response_context,
  const char * service_name,
  const rmw_qos_profile_t * qos)
{
  bool failed;
  auto symbol = get_optional_symbol("rmw_create_service_with_context", failed);
  if (failed) {
    return nullptr;
  }
  if (!symbol) {
    RMW_SET_ERROR_MSG("middleware does not support service message contexts");
    return nullptr;
  }
  return reinterpret_cast<decltype(& rmw_create_service_with_context)>(symbol)(
    node, type_support, request_context, response_context, service_name, qos);
}

rmw_ret_t
rmw_borrow_loaned_request(
  const rmw_client_t * client, const rosidl_runtime_c__message_context_t * context, void ** request)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request, RMW_RET_INVALID_ARGUMENT);
  if (*request) {
    RMW_SET_ERROR_MSG("request output must be null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return call_optional_service_loan<decltype(&rmw_borrow_loaned_request)>(
    "rmw_borrow_loaned_request", client, context, request);
}

rmw_ret_t
rmw_send_loaned_request(const rmw_client_t * client, void ** request, int64_t * sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_send_loaned_request)>(
    "rmw_send_loaned_request", client, request, sequence_id);
}

rmw_ret_t
rmw_return_loaned_request_from_client(const rmw_client_t * client, void ** request)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*request, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_return_loaned_request_from_client)>(
    "rmw_return_loaned_request_from_client", client, request);
}

rmw_ret_t
rmw_take_loaned_response(
  const rmw_client_t * client, const rosidl_runtime_c__message_context_t * context,
  rmw_service_info_t * header, void ** response, bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  if (*response) {
    RMW_SET_ERROR_MSG("response output must be null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return call_optional_service_loan<decltype(&rmw_take_loaned_response)>(
    "rmw_take_loaned_response", client, context, header, response, taken);
}

rmw_ret_t
rmw_return_loaned_response_from_client(const rmw_client_t * client, void ** response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*response, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_return_loaned_response_from_client)>(
    "rmw_return_loaned_response_from_client", client, response);
}

rmw_ret_t
rmw_take_loaned_request(
  const rmw_service_t * service, const rosidl_runtime_c__message_context_t * context,
  rmw_service_info_t * header, void ** request, bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  if (*request) {
    RMW_SET_ERROR_MSG("request output must be null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return call_optional_service_loan<decltype(&rmw_take_loaned_request)>(
    "rmw_take_loaned_request", service, context, header, request, taken);
}

rmw_ret_t
rmw_return_loaned_request_from_service(const rmw_service_t * service, void ** request)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*request, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_return_loaned_request_from_service)>(
    "rmw_return_loaned_request_from_service", service, request);
}

rmw_ret_t
rmw_borrow_loaned_response(
  const rmw_service_t * service, const rosidl_runtime_c__message_context_t * context,
  void ** response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(response, RMW_RET_INVALID_ARGUMENT);
  if (*response) {
    RMW_SET_ERROR_MSG("response output must be null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return call_optional_service_loan<decltype(&rmw_borrow_loaned_response)>(
    "rmw_borrow_loaned_response", service, context, response);
}

rmw_ret_t
rmw_send_loaned_response(
  const rmw_service_t * service, const rmw_request_id_t * header, void ** response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*response, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_send_loaned_response)>(
    "rmw_send_loaned_response", service, header, response);
}

rmw_ret_t
rmw_return_loaned_response_from_service(const rmw_service_t * service, void ** response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(*response, RMW_RET_INVALID_ARGUMENT);
  return call_optional_service_loan<decltype(&rmw_return_loaned_response_from_service)>(
    "rmw_return_loaned_response_from_service", service, response);
}
}  // extern "C"
