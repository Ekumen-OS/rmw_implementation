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

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

TEST(ContextDispatch, fixed_backend_rejects_unsupported_context) {
  rcpputils::SharedLibrary library(
    rcpputils::get_platform_library_name(STRINGIFY(DEFAULT_RMW_IMPLEMENTATION)));
  if (library.has_symbol("rmw_borrow_loaned_message_with_context")) {
    GTEST_SKIP() << "Selected middleware implements message contexts";
  }
  rosidl_runtime_c__message_context_t context{};
  context.constraints.max_total_size = 100;
  EXPECT_EQ(RMW_RET_UNSUPPORTED, rmw_borrow_loaned_message_with_context(
      nullptr, nullptr, &context, nullptr));
  EXPECT_TRUE(rmw_error_is_set());
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_message_with_context(
      nullptr, nullptr, nullptr, nullptr));
  rmw_reset_error();
}
