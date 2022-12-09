// Copyright 2022 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <vector>
#include <string>
#include <sstream>

#include "ros/node_handle.h"

namespace off_highway_common
{

/**
 * \brief Print that parameter key is not available in namespace of node handle.
 *
 * \param key Parameter key
 * \param nh Node handle
 * \return Formatted message
 */
std::string print_missing_key(const std::string & key, const ros::NodeHandle & nh);

/**
 * \brief Print that parameter value is not positive (> 0) in namespace of node handle.
 *
 * \param key Parameter key
 * \param nh Node handle
 * \return Formatted message
 */
std::string print_not_positive(const std::string & key, const ros::NodeHandle & nh);

namespace impl
{
/**
 * \brief Try to get parameter or throw exception.
 *
 * \tparam T Type of parameter
 * \param nh Node handle
 * \param key Parameter key
 * \throw Runtime error if parameter not available.
 * \return Parameter value
 */
template<typename T>
auto get_param_or_throw(ros::NodeHandle & nh, const std::string & key)
{
  T value;
  if (!nh.getParam(key, value)) {
    throw std::runtime_error(off_highway_common::print_missing_key(key, nh));
  }
  return value;
}
}  // namespace impl

/**
 * \brief Try to get parameter or throw exception for signed types.
 *
 * \tparam T Type of parameter
 * \param nh Node handle
 * \param key Parameter key
 * \throw Runtime error if parameter not available.
 * \return Parameter value
 */
template<typename T>
typename std::enable_if<!std::is_unsigned_v<T>, T>::type
get_param_or_throw(ros::NodeHandle & nh, const std::string & key)
{
  if (!nh.hasParam(key)) {
    throw std::runtime_error(print_missing_key(key, nh));
  }

  return impl::get_param_or_throw<T>(nh, key);
}

/**
 * \brief Try to get parameter or throw exception for unsigned types.
 *
 * \tparam T Type of parameter
 * \param nh Node handle
 * \param key Parameter key
 * \throw Runtime error if parameter not available or smaller than zero.
 * \return Parameter value
 */
template<typename T>
typename std::enable_if<std::is_unsigned_v<T>, T>::type
get_param_or_throw(ros::NodeHandle & nh, const std::string & key)
{
  if (!nh.hasParam(key)) {
    throw std::runtime_error(print_missing_key(key, nh));
  }

  int signed_value;
  signed_value = impl::get_param_or_throw<int>(nh, key);
  if (signed_value < 0) {
    throw std::runtime_error(print_not_positive(key, nh));
  }
  return static_cast<T>(signed_value);
}

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

}  // namespace off_highway_common
