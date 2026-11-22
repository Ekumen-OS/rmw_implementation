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

#include <gtest/gtest.h>

#include "rcpputils/shared_library.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

TEST(ContextDispatch, independent_retained_baselines_and_refinements) {
  size_t publisher_specific = 7;
  size_t subscription_specific = 3;
  rosidl_runtime_c__message_context_t publisher_context{};
  publisher_context.constraints.max_total_size = 100;
  publisher_context.constraints.type_specific = &publisher_specific;
  rosidl_runtime_c__message_context_t subscription_context{};
  subscription_context.constraints.max_total_size = 50;
  subscription_context.constraints.type_specific = &subscription_specific;
  auto publisher = rmw_create_publisher_with_context(
    nullptr, nullptr, &publisher_context, nullptr, nullptr, nullptr);
  auto subscription = rmw_create_subscription_with_context(
    nullptr, nullptr, &subscription_context, nullptr, nullptr, nullptr);
  ASSERT_NE(nullptr, publisher);
  ASSERT_NE(nullptr, subscription);
  publisher_context.constraints.max_total_size = 1;
  subscription_context.constraints.max_total_size = 1;
  publisher_specific = 1;
  subscription_specific = 1;

  size_t requested_specific = 4;
  rosidl_runtime_c__message_context_t request{};
  request.constraints.max_total_size = 75;
  request.constraints.type_specific = &requested_specific;
  void * message = nullptr;
  EXPECT_EQ(RMW_RET_OK, rmw_borrow_loaned_message_with_context(
      publisher, nullptr, &request, &message));
  bool taken = false;
  rmw_message_info_t info{};
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_take_loaned_message_with_info_and_context(
      subscription, &request, &message, &taken, &info, nullptr));
  rmw_reset_error();

  request.constraints.max_total_size = 40;
  requested_specific = 2;
  ASSERT_EQ(RMW_RET_OK, rmw_take_loaned_message_with_info_and_context(
      subscription, &request, &message, &taken, &info, nullptr));
  EXPECT_TRUE(taken);
  EXPECT_EQ(42, info.source_timestamp);
  request.constraints.max_total_size = 0;
  requested_specific = 4;
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_take_loaned_message_with_info_and_context(
      subscription, &request, &message, &taken, &info, nullptr));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_OK, rmw_take_loaned_message_with_info_and_context(
      subscription, nullptr, &message, &taken, &info, nullptr));
  EXPECT_EQ(RMW_RET_OK, rmw_take_loaned_message_with_context(
      subscription, nullptr, &message, &taken, nullptr));

  rcpputils::SharedLibrary backend(
    rcpputils::get_platform_library_name("rmw_context_test_backend"));
  auto set_sample_size = reinterpret_cast<void (*)(size_t)>(
    backend.get_symbol("context_test_set_sample_size"));
  set_sample_size(60);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_take_loaned_message_with_info_and_context(
      subscription, nullptr, &message, &taken, &info, nullptr));
  rmw_reset_error();
}

TEST(ContextDispatch, service_loan_identity_constraints_and_consumption) {
  rosidl_runtime_c__message_context_t request_context{};
  rosidl_runtime_c__message_context_t response_context{};
  request_context.constraints.max_total_size = 100;
  response_context.constraints.max_total_size = 50;
  auto client = rmw_create_client_with_context(
    nullptr, nullptr, &request_context, &response_context, nullptr, nullptr);
  auto service = rmw_create_service_with_context(
    nullptr, nullptr, &request_context, &response_context, nullptr, nullptr);
  ASSERT_NE(nullptr, client);
  ASSERT_NE(nullptr, service);
  request_context.constraints.max_total_size = 1;
  response_context.constraints.max_total_size = 1;

  rosidl_runtime_c__message_context_t refinement{};
  refinement.constraints.max_total_size = 75;
  void * request = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_request(client, &refinement, &request));
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_request(client, nullptr, &request));
  rmw_reset_error();
  int64_t sequence = 0;
  ASSERT_EQ(RMW_RET_OK, rmw_send_loaned_request(client, &request, &sequence));
  EXPECT_EQ(nullptr, request);
  EXPECT_EQ(42, sequence);
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_request(client, nullptr, &request));
  ASSERT_EQ(RMW_RET_OK, rmw_return_loaned_request_from_client(client, &request));
  EXPECT_EQ(nullptr, request);

  rmw_service_info_t header{};
  bool taken = false;
  ASSERT_EQ(RMW_RET_OK, rmw_take_loaned_request(service, &refinement, &header, &request, &taken));
  ASSERT_TRUE(taken);
  EXPECT_EQ(sequence, header.request_id.sequence_number);
  ASSERT_EQ(RMW_RET_OK, rmw_return_loaned_request_from_service(service, &request));
  EXPECT_EQ(nullptr, request);

  void * response = nullptr;
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_response(service, &refinement, &response));
  EXPECT_EQ(nullptr, response);
  rmw_reset_error();
  refinement.constraints.max_total_size = 40;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_response(service, &refinement, &response));
  rmw_request_id_t wrong = header.request_id;
  wrong.sequence_number = 0;
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_send_loaned_response(service, &wrong, &response));
  EXPECT_NE(nullptr, response);
  rmw_reset_error();
  ASSERT_EQ(RMW_RET_OK, rmw_send_loaned_response(service, &header.request_id, &response));
  EXPECT_EQ(nullptr, response);
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_response(service, nullptr, &response));
  ASSERT_EQ(RMW_RET_OK, rmw_return_loaned_response_from_service(service, &response));
  EXPECT_EQ(nullptr, response);
  taken = false;
  ASSERT_EQ(RMW_RET_OK, rmw_take_loaned_response(client, &refinement, &header, &response, &taken));
  EXPECT_TRUE(taken);
  EXPECT_EQ(sequence, header.request_id.sequence_number);
  ASSERT_EQ(RMW_RET_OK, rmw_return_loaned_response_from_client(client, &response));
  EXPECT_EQ(nullptr, response);
}
