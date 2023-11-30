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

#include "off_highway_can/helper.hpp"

namespace off_highway_can
{

std::string print_missing_key(const std::string & key, const ros::NodeHandle & nh)
{
  std::stringstream msg;
  msg << "Parameter '" << key << "' in namespace '" << nh.getNamespace() << "' missing!";
  ROS_FATAL_STREAM(msg.str());
  return msg.str();
}

std::string print_not_positive(const std::string & key, const ros::NodeHandle & nh)
{
  std::stringstream msg;
  msg << "Parameter '" << key << "' in namespace '" << nh.getNamespace() <<
    "' is larger than zero!";
  ROS_FATAL_STREAM(msg.str());
  return msg.str();
}

}  // namespace off_highway_can
