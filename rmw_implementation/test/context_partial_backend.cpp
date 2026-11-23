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

#include "rmw/rmw.h"

extern "C"
RMW_PUBLIC
rmw_publisher_t *
rmw_create_publisher_with_context(
  const rmw_node_t *, const rosidl_message_type_support_t *,
  const rosidl_runtime_c__message_context_t *, const char *, const rmw_qos_profile_t *,
  const rmw_publisher_options_t *)
{
  return nullptr;
}
