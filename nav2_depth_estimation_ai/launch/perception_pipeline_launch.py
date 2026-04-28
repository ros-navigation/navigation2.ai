from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    nav2_depth_ai_dir = get_package_share_directory("nav2_depth_estimation_ai")

    # Create the launch configuration variables
    container_name = LaunchConfiguration("container_name")
    params_file = LaunchConfiguration("params_file")
    use_intra_process_comms = LaunchConfiguration("use_intra_process_comms")
    use_usb_cam = LaunchConfiguration("use_usb_cam")
    use_depth_anything = LaunchConfiguration("use_depth_anything")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [nav2_depth_ai_dir, "params", "nav2_depth_ai_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_usb_cam_cmd = DeclareLaunchArgument(
        "use_usb_cam",
        default_value="True",
        description="Whether to use Usb Cam",
    )

    declare_use_depth_anything_cmd = DeclareLaunchArgument(
        "use_depth_anything",
        default_value="True",
        description="Whether to use Depth Anything",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        "use_intra_process_comms",
        default_value="True",
        description="Whether to use intra process communications",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_depth_ai_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="usb_cam",
                name="usb_cam",
                executable="usb_cam_exe",
                output="screen",
                respawn=use_respawn,
                parameters=[params_file],
                remapping=[
                    ("/camera_info", "/pipeline/camera_info"),
                    ("/image_raw", "/pipeline/image_raw"),
                ],
                arguments=["--ros-args", "--log-level", log_level],
                condition=IfCondition(use_usb_cam),
            ),
            Node(
                package="image_proc",
                name="crop_decimate",
                executable="crop_decimate_node",
                output="screen",
                respawn=use_respawn,
                parameters=[params_file],
                remapping=[
                    ("/in/camera_info", "/pipeline/camera_info"),
                    ("/in/image_raw", "/pipeline/image_raw"),
                    ("/out/camera_info", "/image_crop_decimate/camera_info"),
                    ("/out/image_raw", "/image_crop_decimate/image"),
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="image_proc",
                name="resize",
                executable="resize_node",
                output="screen",
                respawn=use_respawn,
                parameters=[params_file],
                remapping=[
                    ("/image/camera_info", "/image_crop_decimate/camera_info"),
                    ("/image/image_raw", "/image_crop_decimate/image_raw"),
                    # NOTE: this camera_info topic is not remapping
                    # (
                    #     "/resize/camera_info",
                    #     "/pipeline/camera_info_preprocessed",
                    # ),
                    # ("/resize/image_raw", "/pipeline/image_preprocessed"),
                    # (
                    #     "/resize/image_raw/compressed",
                    #     "/pipeline/image_preprocessed/compressed",
                    # ),
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="depth_anything_v3",
                name="depth_anything_v3",
                executable="depth_anything_v3_exe",
                output="screen",
                respawn=use_respawn,
                parameters=[params_file],
                remapping=[
                    ("~/input/camera_info", "/resize/camera_info"),
                    ("~/input/image", "/resize/image_raw/compressed"),
                    ("~/output/depth_image", "depth_image"),
                ],
                arguments=["--ros-args", "--log-level", log_level],
                condition=IfCondition(use_depth_anything),
            ),
            Node(
                package="depth_image_proc",
                name="pointcloud",
                executable="point_cloud_xyzrgb_node",
                output="screen",
                respawn=use_respawn,
                parameters=[params_file],
                remapping=[
                    ("rgb/camera_info", "/resize/camera_info"),
                    ("rgb/image_rect_color", "/resize/image_raw"),
                    ("depth_registered/image_rect", "depth_image"),
                    ("points", "/pipeline/points"),
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package="usb_cam",
                        plugin="usb_cam::UsbCamNode",
                        name="usb_cam",
                        parameters=[params_file],
                        remappings=[
                            ("/camera_info", "/pipeline/camera_info"),
                            ("/image_raw", "/pipeline/image_raw"),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": use_intra_process_comms}
                        ],
                        condition=IfCondition(use_usb_cam),
                    ),
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::CropDecimateNode",
                        name="crop_decimate",
                        parameters=[params_file],
                        remappings=[
                            ("/in/camera_info", "/pipeline/camera_info"),
                            ("/in/image_raw", "/pipeline/image_raw"),
                            ("/out/camera_info", "/image_crop_decimate/camera_info"),
                            ("/out/image_raw", "/image_crop_decimate/image"),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": use_intra_process_comms}
                        ],
                    ),
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::ResizeNode",
                        name="resize",
                        parameters=[params_file],
                        remappings=[
                            ("/image/camera_info", "/image_crop_decimate/camera_info"),
                            ("/image/image_raw", "/image_crop_decimate/image"),
                            # NOTE: this camera_info topic is not remapping
                            # (
                            #     "/resize/camera_info",
                            #     "/pipeline/camera_info_preprocessed",
                            # ),
                            # ("/resize/image_raw", "/pipeline/image_preprocessed"),
                            # (
                            #     "/resize/image_raw/compressed",
                            #     "/pipeline/image_preprocessed/compressed",
                            # ),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": use_intra_process_comms}
                        ],
                    ),
                    ComposableNode(
                        package="depth_anything_v3",
                        plugin="depth_anything_v3::DepthAnythingV3Node",
                        name="depth_anything_v3",
                        parameters=[params_file],
                        remappings=[
                            ("~/input/camera_info", "/resize/camera_info"),
                            ("~/input/image", "/resize/image_raw/compressed"),
                            ("~/output/depth_image", "depth_image"),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": use_intra_process_comms}
                        ],
                        condition=IfCondition(use_depth_anything),
                    ),
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="pointcloud",
                        parameters=[params_file],
                        remappings=[
                            ("rgb/camera_info", "/resize/camera_info"),
                            ("rgb/image_rect_color", "/resize/image_raw"),
                            ("depth_registered/image_rect", "depth_image"),
                            ("points", "/pipeline/points"),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": use_intra_process_comms}
                        ],
                    ),
                ],
            )
        ],
    )

    container = Node(
        package="rclcpp_components",
        executable="component_container",
        name=container_name,
        output="screen",
        parameters=[{"use_intra_process_comms": use_intra_process_comms}],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_composition),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_usb_cam_cmd)
    ld.add_action(declare_use_depth_anything_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_intra_process_comms_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch
    ld.add_action(load_nodes)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)

    return ld
