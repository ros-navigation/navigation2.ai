import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    cfg = load_yaml(
        os.path.join(
            get_package_share_directory('nav2_perception_pipelines'),
            'config',
            'perception_pipeline.yaml'
        )
    )

    composable_nodes = [

        # 1) Image source
        ComposableNode(
            name='image_source',
            package=cfg['image_source']['package'],
            plugin=cfg['image_source']['plugin'],
            parameters=[cfg['image_source'].get('parameters', {})],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

        # 2) Image preprocessor
        ComposableNode(
            name='image_preprocessor',
            package=cfg['image_preprocessor']['package'],
            plugin=cfg['image_preprocessor']['plugin'],
            parameters=[cfg['image_preprocessor'].get('parameters', {})],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

        # 3) Depth estimator
        ComposableNode(
            name='depth_estimator',
            package=cfg['depth_estimator']['package'],
            plugin=cfg['depth_estimator']['plugin'],
            parameters=[cfg['depth_estimator'].get('parameters', {})],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

        # 4) PointCloud projection
        ComposableNode(
            name='pointcloud_projection',
            package=cfg['pointcloud_projection']['package'],
            plugin=cfg['pointcloud_projection']['plugin'],
            parameters=[cfg['pointcloud_projection'].get('parameters', {})],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription([container])
