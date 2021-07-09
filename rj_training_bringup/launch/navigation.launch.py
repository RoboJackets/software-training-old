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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True'
        ),
        DeclareLaunchArgument(
            name="navigation_params_file",
            default_value=os.path.join(get_package_share_directory(
                'rj_training_bringup'), 'config', 'nav_params.yaml')
        ),
        DeclareLaunchArgument(
            name="behavior_tree_file",
            default_value=os.path.join(get_package_share_directory(
                'rj_training_bringup'), 'behavior_trees', 'navigate.xml')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'False',
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': LaunchConfiguration('navigation_params_file'),
                              'default_bt_xml_filename': LaunchConfiguration('behavior_tree_file'),
                              'autostart': 'True'}.items())
    ])
