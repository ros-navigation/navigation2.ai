# Copyright (c) 2026 Ocean Code AI Ltd
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
#
# NOTE: Authored with AI assistance, reviewed and validated by the submitter.
# Disclosed per the Nav2 project's AI-generated-software policy.
#
# Loads the depth filter as a composable node into an existing container (by
# default the nav2_depth_estimation_ai pipeline container), between the depth
# estimator and the point-cloud projection.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = LaunchConfiguration('container_name')
    intra = LaunchConfiguration('use_intra_process_comms')

    return LaunchDescription([
        DeclareLaunchArgument('container_name', default_value='nav2_depth_ai_container'),
        DeclareLaunchArgument('use_intra_process_comms', default_value='True'),
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_depth_filter',
                    plugin='nav2_depth_filter::DepthFilterNode',
                    name='depth_filter',
                    parameters=[{
                        'enabled': True,
                        'min_depth': 0.2,
                        'max_depth': 20.0,
                        'median_ksize': 5,
                        'speckle_max_diff': 0.5,
                        'min_region': 40,
                        'flying_thresh': 0.0,
                    }],
                    remappings=[
                        ('~/input/depth_image', '/depth_image'),
                        ('~/output/depth_image', '/depth_image_filtered'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra}],
                ),
            ],
        ),
    ])
