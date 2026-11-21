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
