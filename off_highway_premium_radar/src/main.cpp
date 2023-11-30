// Copyright 2023 Robert Bosch GmbH and its subsidiaries
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

#include "rclcpp/rclcpp.hpp"

#include "off_highway_premium_radar/node.hpp"
#include "off_highway_premium_radar/converters/default_converter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  off_highway_premium_radar::Node::Converters converters;
  converters.emplace_back(std::make_shared<off_highway_premium_radar::DefaultConverter>());

  auto node = std::make_shared<off_highway_premium_radar::Node>(
    "off_highway_premium_radar_driver");
  node->configure(converters);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
