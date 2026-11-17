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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

share_dir = get_package_share_directory('zed_wrapper')
default_override_path = os.path.join(share_dir, 'config', 'zed2i_override.yaml')


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    overrides = {
        'video.publish_left_right': arg('left_right'),
        'video.publish_stereo': arg('stereo'),
    }
    # NONE also stops the positional tracking, which needs the depth.
    if arg('depth') != 'true':
        overrides['depth.depth_mode'] = 'NONE'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': arg('camera_name'),
                'serial_number': arg('serial_number'),
                'ros_params_override_path': arg('ros_params_override_path'),
                'publish_urdf': arg('publish_urdf'),
                'publish_tf': arg('publish_tf'),
                'param_overrides': ';'.join(k + ':=' + v for k, v in overrides.items()),
            }.items()
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_name',
                default_value='zed2i',
                description='The name of the camera, used as node namespace.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to open. 0 opens the first camera found.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value=default_override_path,
                description='The path to an additional parameters file to override the default values. Default: config/zed2i_override.yaml'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.'),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> camera_link` TF.'),
            DeclareLaunchArgument(
                'left_right',
                default_value='true',
                description='Advertise the left and right image topics.'),
            DeclareLaunchArgument(
                'stereo',
                default_value='true',
                description='Advertise the side-by-side stereo image topics.'),
            DeclareLaunchArgument(
                'depth',
                default_value='false',
                description='Compute the depth: the depth map, the point cloud and the positional tracking. false sets depth.depth_mode NONE.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
