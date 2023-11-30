// Copyright 2022 Robert Bosch GmbH and its subsidiaries
// Copyright 2023 digital workbench GmbH
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

#pragma once

namespace off_highway_can
{
/**
 * \brief Static cast to deduced type.
 *
 * \tparam To Cast target type
 * \tparam From Cast from type
 * \param to Variable to hold casted type
 * \param from Variable to cast from
 */
template<typename To, typename From>
void auto_static_cast(To & to, const From & from)
{
  to = static_cast<To>(from);
}

}  // namespace off_highway_can
