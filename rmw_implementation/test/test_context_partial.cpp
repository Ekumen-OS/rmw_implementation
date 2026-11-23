// Copyright 2026 Ekumen Inc.
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

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

TEST(ContextDispatch, partial_region_protocol_is_unavailable) {
  for (auto role : {
    RMW_REGION_LOAN_PUBLISHER, RMW_REGION_LOAN_SUBSCRIPTION,
    RMW_REGION_LOAN_CLIENT, RMW_REGION_LOAN_SERVICE})
  {
    bool available = true;
    ASSERT_EQ(RMW_RET_OK, rmw_region_loan_entry_points_available(role, &available));
    EXPECT_FALSE(available);
  }
}

TEST(ContextDispatch, missing_service_capability_query_is_unavailable) {
  rmw_client_t client{};
  rmw_service_t service{};
  bool available = true;
  EXPECT_EQ(RMW_RET_OK, rmw_region_client_can_loan_messages(&client, &available));
  EXPECT_FALSE(available);
  available = true;
  EXPECT_EQ(RMW_RET_OK, rmw_region_service_can_loan_messages(&service, &available));
  EXPECT_FALSE(available);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_region_client_can_loan_messages(nullptr, &available));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_region_service_can_loan_messages(&service, nullptr));
  rmw_reset_error();
}

TEST(ContextDispatch, missing_type_capability_query_is_unavailable) {
  rosidl_message_type_support_t type{};
  bool available = true;
  EXPECT_EQ(
    RMW_RET_OK, rmw_region_loan_type_support_available(
      RMW_REGION_LOAN_PUBLISHER, &type, nullptr, nullptr, nullptr, &available));
  EXPECT_FALSE(available);
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT, rmw_region_loan_type_support_available(
      RMW_REGION_LOAN_CLIENT, &type, nullptr, nullptr, nullptr, &available));
  rmw_reset_error();
}
