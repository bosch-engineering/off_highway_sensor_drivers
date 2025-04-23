# Copyright 2024 Robert Bosch GmbH and its subsidiaries
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    off_highway_premium_radar_params = PathJoinSubstitution([
        FindPackageShare('off_highway_premium_radar'),
        'config',
        'driver.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('off_highway_sensor_drivers_examples'),
        'config',
        'off_highway_premium_radar_filter.rviz'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('off_highway_premium_radar_params',
                              default_value=off_highway_premium_radar_params,
                              description='Parameters for premium radar driver'),
        DeclareLaunchArgument('rviz_config',
                              default_value=rviz_config,
                              description='rviz configuration file'),
        ComposableNodeContainer(
            name='filtered_off_highway_premium_radar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='off_highway_premium_radar',
                    plugin='off_highway_premium_radar::NodeWithDefaultConverter',
                    name='off_highway_premium_radar_driver',
                    parameters=[LaunchConfiguration('off_highway_premium_radar_params')],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name='pcl_filter_z',
                    remappings=[
                        ('/input', '/off_highway_premium_radar_driver/locations'),
                        ('/output', '/filter/z_filtered'),
                    ],
                    parameters=[
                        {'filter_field_name': 'z'},
                        {'filter_limit_min': 0.0},
                        {'filter_limit_max': 4.0},
                        {'input_frame': 'base_link'},
                        {'output_frame': 'base_link'},
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name='pcl_filter_x',
                    remappings=[
                        ('/input', '/filter/z_filtered'),
                        ('/output', '/filter/x_filtered'),
                    ],
                    parameters=[
                        {'filter_field_name': 'x'},
                        {'filter_limit_min': 0.0},
                        {'filter_limit_max': 30.0},
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name='pcl_filter_rcs',
                    remappings=[
                        ('/input', '/filter/x_filtered'),
                        ('/output', '/filter/rcs_filtered'),
                    ],
                    parameters=[
                        {'filter_field_name': 'radar_cross_section'},
                        {'filter_limit_min': -8.0},
                        {'filter_limit_max': 30.0},
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
