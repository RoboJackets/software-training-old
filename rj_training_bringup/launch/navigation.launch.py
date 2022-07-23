# Copyright 2021 RoboJackets
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']

    default_behavior_tree = os.path.join(get_package_share_directory(
        'rj_training_bringup'), 'behavior_trees', 'navigate.xml')

    params_file = LaunchConfiguration('nav_params_file')

    use_sim_time = LaunchConfiguration('use_sim_time')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': LaunchConfiguration('behavior_tree'),
        'default_nav_through_poses_bt_xml': LaunchConfiguration('behavior_tree')
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='nav_params_file',
            default_value=os.path.join(get_package_share_directory(
                'rj_training_bringup'), 'config', 'nav_params.yaml')
        ),
        DeclareLaunchArgument(
            name='behavior_tree',
            default_value=default_behavior_tree
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        )
    ])
