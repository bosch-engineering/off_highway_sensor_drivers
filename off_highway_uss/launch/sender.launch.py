# Copyright 2022 Robert Bosch GmbH and its subsidiaries
# Copyright 2023 digital workbench GmbH
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

import ament_index_python.packages
import launch
import launch_ros.actions
import pathlib


def generate_launch_description():
    off_highway_uss = ament_index_python.packages.\
        get_package_share_directory('off_highway_uss')
    uss_params = pathlib.Path(off_highway_uss, 'config', 'sender_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='off_highway_uss',
            executable='sender',
            name='off_highway_uss_sender',
            output='screen',
            parameters=[uss_params],
            arguments=[]
        )
    ])
