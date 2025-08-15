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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    occ_model_file_path = os.path.join(
        get_package_share_directory("dstereo_occnet"),
        "config",
        "X5-OCC-32x64x96x2_constinput_modified.bin",
    )

    web_pub_arg = DeclareLaunchArgument(
        "web_pub",
        default_value="true",
        description="Enable web publishing. If false, websocket and codec nodes will be disabled.",
    )

    # zero-copy environment configuration
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # OccNet algorithm node
    dstereo_occnet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dstereo_occnet"),
                "launch/dstereo_occnet_node.launch.py",
            )
        ),
        launch_arguments={
            "use_local_image": "true",
            "occ_model_file_path": occ_model_file_path,
        }.items(),
    )

    # codec node
    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_encode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_out_mode": "ros",
            "codec_sub_topic": "/image_combine_raw",
            "codec_in_format": "nv12",
            "codec_pub_topic": "/image_jpeg",
            "codec_out_format": "jpeg",
            "log_level": "warn",
        }.items(),
        condition=IfCondition(LaunchConfiguration("web_pub")),
    )

    # web node
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image_jpeg",
            "websocket_only_show_image": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("web_pub")),
    )

    return LaunchDescription(
        [
            web_pub_arg,
            shared_mem_node,
            dstereo_occnet_node,
            codec_node,
            web_node,
        ]
    )
