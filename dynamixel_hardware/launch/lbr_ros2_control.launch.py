# Copyright (c) 2025 Tohoku Univ. Space Robotics Lab.
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

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg_share = get_package_share_directory('lbr_description')
    utils_path = os.path.join(desc_pkg_share, 'launch')
    hw_pkg_share = get_package_share_directory('dynamixel_hardware')

    if utils_path not in sys.path:
        sys.path.append(utils_path)

    from lbr_launch_utils import get_limbero_config, get_limbero_description

    # Import robot config from yaml
    config = get_limbero_config()
    is_grieel_model = config['is_grieel_model']
    limb_names = config['limb_names']

    suffix = '_grieel' if is_grieel_model else ''

    launch_entities = []

    for limb in limb_names:
        robot_desc_xml = get_limbero_description(ros2_control_limb=limb)

        # Controller manager config for each limb
        controller_config_path = os.path.join(
            hw_pkg_share,
            'config',
            f'{limb}{suffix}_config.yaml'
        )

        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=limb,
            parameters=[
                {'robot_description': robot_desc_xml},
                controller_config_path
            ],
            output={'stdout': 'screen', 'stderr': 'screen'},
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=limb,
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_desc_xml}],
            output='screen',
            remappings=[(f'/{limb}/joint_states',
                        f'/{limb}/serial_link_joint_states')]
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=limb,
            arguments=['joint_state_broadcaster',
                       '-c',
                       f'/{limb}/controller_manager'
                       ],
            output='screen'
        )

        joint_trajectory_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=limb,
            arguments=['joint_trajectory_controller',
                       '-c',
                       f'/{limb}/controller_manager'
                       ],
            output='screen'
        )

        gripper_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=limb,
            arguments=['gripper_controller',
                       '-c',
                       f'/{limb}/controller_manager'
                       ],
            output='screen'
        )

        reboot_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=limb,
            arguments=['reboot_controller',
                       '-c',
                       f'/{limb}/controller_manager'
                       ],
            output='screen'
        )

        delay_spawners_after_control_node = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    joint_state_broadcaster_spawner,
                    joint_trajectory_controller_spawner,
                    gripper_controller_spawner,
                    reboot_controller_spawner
                ]
            )
        )

        # Add each node to launch lists
        launch_entities.append(ros2_control_node)
        launch_entities.append(robot_state_publisher_node)
        launch_entities.append(delay_spawners_after_control_node)

    return LaunchDescription(launch_entities)
