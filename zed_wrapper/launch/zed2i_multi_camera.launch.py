# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

share_dir = get_package_share_directory('zed_wrapper')

# Camera list: model, serials and start interval
default_cameras_path = os.path.join(share_dir, 'config', 'zed2i_multi_camera.yaml')

# Parameter override applied to every camera
default_override_path = os.path.join(share_dir, 'config', 'zed2i_multi_override.yaml')


def parse_array_param(param):
    cleaned = param.replace('[', '').replace(']', '').replace(' ', '')
    if not cleaned:
        return []
    return cleaned.split(',')


def launch_setup(context, *args, **kwargs):
    with open(default_cameras_path) as f:
        cameras = yaml.safe_load(f)

    serials = parse_array_param(LaunchConfiguration('serials').perform(context))
    if not serials:
        serials = [str(sn) for sn in cameras['serials']]
    stagger = float(cameras['stagger'])

    actions = []
    for i, sn in enumerate(serials):
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_model': cameras['camera_model'],
                'camera_name': 'zed2i_' + str(i),
                'serial_number': sn,
                'ros_params_override_path': default_override_path
            }.items()
        )
        actions.append(TimerAction(period=i * stagger, actions=[include]))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serials',
                default_value='',
                description='Serial numbers of the cameras to open. Default: the list in config/zed2i_multi_camera.yaml'),
            OpaqueFunction(function=launch_setup)
        ]
    )
