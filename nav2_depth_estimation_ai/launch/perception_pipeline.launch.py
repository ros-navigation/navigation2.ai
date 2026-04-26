import os
import yaml

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# -------------------------------
# Internal pipeline topic contracts
# -------------------------------
PIPELINE_IMAGE_RAW = '/pipeline/image_raw'
PIPELINE_CAMERA_INFO = '/pipeline/camera_info'
PIPELINE_IMAGE_PREPROCESSED = '/pipeline/image_preprocessed'
PIPELINE_CAMERA_INFO_PREPROCESSED = '/pipeline/camera_info_preprocessed'
PIPELINE_IMAGE_PREPROCESSED_COMPRESSED = '/pipeline/image_preprocessed/compressed'
PIPELINE_IMAGE_DEPTH = '/pipeline/depth'

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():

    cfg = load_yaml(
        os.path.join(
            get_package_share_directory('nav2_depth_estimation_ai'),
            'config',
            'perception_pipeline.yaml'
        )
    )
    
    nodes = []
    launch_actions = []

    source_type = cfg['image_source'].get('type')
    preprocessing_enabled = cfg.get('image_preprocessor', {}).get('enabled', True)
    depth_enabled = cfg.get('depth_estimator', {}).get('enabled', True)

    if source_type is None:
        raise RuntimeError(
            "Missing required field 'image_source.type' in configuration. "
            "It must be set to either 'rgb' or 'depth'."
        )

    if source_type not in ('rgb', 'depth'):
        raise RuntimeError(
            f"Invalid image_source.type='{source_type}'. "
            "Allowed values are: 'rgb' or 'depth'."
        )
        
    if source_type == 'rgb' and not depth_enabled:
        launch_actions.append(
            LogInfo(
                msg=(
                    "[WARNING] Projection requires depth, but "
                    "image_source.type='rgb' and depth_estimator is disabled."
                )
            )
        )

    # ----------------------------------
    # 1) Image Source - likely your camera driver (realsense, ML camera, etc)
    # ----------------------------------
    nodes.append(
        ComposableNode(
            name='image_source',
            package=cfg['image_source']['package'],
            plugin=cfg['image_source']['plugin'],
            parameters=[cfg['image_source'].get('parameters', {})],
            remappings=[
                (cfg['image_source']['topics']['output_topic'], PIPELINE_IMAGE_RAW),
                (cfg['image_source']['topics']['camera_info_topic'], PIPELINE_CAMERA_INFO),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    )

    # ----------------------------------
    # 2) Image Preprocessing
    # ----------------------------------
    if preprocessing_enabled:
        nodes.append(
            ComposableNode(
                name='image_crop_decimate',
                package='image_proc',
                plugin='image_proc::CropDecimateNode',
                parameters=[
                    cfg['image_preprocessor']['parameters']['crop_decimate']
                ],
                remappings=[
                    ('/in/image_raw', PIPELINE_IMAGE_RAW),
                    ('/in/camera_info', PIPELINE_CAMERA_INFO),
                    ('/out/image_raw', '/image_crop_decimate/image'),
                    ('/out/camera_info', '/image_crop_decimate/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

        nodes.append(
            ComposableNode(
                name='image_resize',
                package='image_proc',
                plugin='image_proc::ResizeNode',
                parameters=[
                    cfg['image_preprocessor']['parameters']['resize']
                ],
                remappings=[
                    ('/image/image_raw', '/image_crop_decimate/image'),
                    ('/image/camera_info', '/image_crop_decimate/camera_info'),
                    ('/resize/image_raw', PIPELINE_IMAGE_PREPROCESSED),
                    ('/resize/image_raw/compressed', PIPELINE_IMAGE_PREPROCESSED_COMPRESSED),
                    ('/resize/camera_info', PIPELINE_CAMERA_INFO_PREPROCESSED)
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

        depth_input_image = PIPELINE_IMAGE_PREPROCESSED_COMPRESSED
        depth_input_camera_info = PIPELINE_CAMERA_INFO_PREPROCESSED

    else:
        depth_input_image = PIPELINE_IMAGE_RAW
        depth_input_camera_info = PIPELINE_CAMERA_INFO

    # ----------------------------------
    # 3) Depth Estimator
    # ----------------------------------
    if depth_enabled and (source_type=='rgb') :

        depth_cfg = cfg['depth_estimator']
        model_path_cfg = depth_cfg['model_path']
        if os.path.isabs(model_path_cfg):
            absolute_model_path = model_path_cfg
        else:
            pkg_share = get_package_share_directory(depth_cfg['package'])
            absolute_model_path = os.path.join(pkg_share, model_path_cfg)

        if not os.path.exists(absolute_model_path):
            raise RuntimeError(
                f"[DepthEstimator] Model file not found: {absolute_model_path}"
            )

        depth_params = depth_cfg.get('parameters', {}).copy()
        depth_params[depth_cfg.get('model_path_name_argument')] = absolute_model_path

        depth_topics = depth_cfg['topics']

        nodes.append(
            ComposableNode(
                name='depth_estimator',
                package=depth_cfg['package'],
                plugin=depth_cfg['plugin'],
                parameters=[depth_params],
                remappings=[
                    (depth_topics['input_image'], depth_input_image),
                    (depth_topics['input_camera_info'], depth_input_camera_info),
                    (depth_topics['output_depth'], PIPELINE_IMAGE_DEPTH),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

        image_for_projection = PIPELINE_IMAGE_DEPTH

    else:
        image_for_projection = depth_input_image

    # ----------------------------------
    # 4) PointCloud Projection
    # ----------------------------------
    nodes.append(
        ComposableNode(
            name='pointcloud_projection',
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            parameters=[{
                'queue_size': 10,
                'approximate_sync': False,
                'use_sim_time': False
            }],
            remappings=[
                ('image_rect', image_for_projection),
                ('camera_info', depth_input_camera_info),
                ('points', '/pipeline/points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    )

    # ----------------------------------
    # Container
    # ----------------------------------
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    return LaunchDescription(launch_actions + [container])

