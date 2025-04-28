# Copyright 2025 Robert Bosch GmbH and its subsidiaries
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('off_highway_premium_radar'),
        'config',
        'driver.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('params',
                              default_value=params,
                              description='Parameters for driver'),
        Node(
            package='off_highway_premium_radar',
            executable='driver',
            name='off_highway_premium_radar_driver',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params')]
        )
    ])
