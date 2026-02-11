# Copyright 2025 The ads1115_adc Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch file for ads1115_adc package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ADS1115 ADC node."""
    pkg_dir = get_package_share_directory('ads1115_adc')
    default_params = os.path.join(pkg_dir, 'config', 'ads1115_params.yaml')

    return LaunchDescription([
        # ── Launch arguments (override on CLI) ────────
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

        # ── ADS1115 ADC Node ─────────────────────────
        Node(
            package='ads1115_adc',
            executable='ads1115_node.py',
            name='ads1115_adc_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
