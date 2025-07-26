# Copyright (c) 2025，D-Robotics.
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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default_value"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def generate_launch_description():

    occ_model_file_path = os.path.join(
        get_package_share_directory("dstereo_occnet"),
        "config",
        "X5-OCC-32x64x96x2_constinput_modified.bin",
    )

    local_image_dir = os.path.join(
        get_package_share_directory("dstereo_occnet"), "config"
    )

    node_params = [
        {
            "name": "stereo_msg_topic",
            "default_value": "/image_combine_raw",
            "description": "stereo_msg_topic",
        },
        {
            "name": "camera_info_topic",
            "default_value": "/image_combine_raw/camera_info",
            "description": "camera_info_topic",
        },
        {
            "name": "occ_model_file_path",
            "default_value": occ_model_file_path,
            "description": "occ_model_file_path",
        },
        {
            "name": "use_local_image",
            "default_value": "false",
            "description": "use_local_image",
        },
        {
            "name": "local_image_dir",
            "default_value": local_image_dir,
            "description": "local_image_dir",
        },
        {
            "name": "save_occ_flag",
            "default_value": "false",
            "description": "save_occ_flag",
        },
        {
            "name": "save_occ_dir",
            "default_value": "./occ_results",
            "description": "save_occ_dir",
        },
        {
            "name": "save_freq",
            "default_value": "1",
            "description": "save_freq",
        },
        {
            "name": "save_total",
            "default_value": "-1",
            "description": "save_total",
        },
        {
            "name": "voxel_size",
            "default_value": "0.02",
            "description": "voxel_size",
        },
        {"name": "log_level", "default_value": "info", "description": "log_level"},
    ]

    launch = declare_configurable_parameters(node_params)
    launch.append(
        Node(
            package="dstereo_occnet",
            executable="dstereo_occnet_node",
            output="screen",
            parameters=[set_configurable_parameters(node_params)],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    )

    return LaunchDescription(launch)
