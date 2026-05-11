import os, sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    EnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "rbl_controller_node"
    pkg_path = get_package_share_directory(pkg_name)

    default_config_path = os.path.join(pkg_path, "config", "default.yaml")

    # -------------------------------------------------
    # Launch configurations
    # -------------------------------------------------
    uav_name = LaunchConfiguration("uav_name")
    standalone = LaunchConfiguration("standalone")
    container_name = LaunchConfiguration("container_name")
    custom_config = LaunchConfiguration("custom_config")
    pcl_topic = LaunchConfiguration("pcl_topic")  # 👈 NEW
    debug = LaunchConfiguration("debug")

    # -------------------------------------------------
    # Launch description + arguments
    # -------------------------------------------------
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_name",
                default_value=EnvironmentVariable("UAV_NAME", default_value="uav1"),
            ),
            DeclareLaunchArgument("standalone", default_value="true"),
            DeclareLaunchArgument("container_name", default_value=""),
            DeclareLaunchArgument(
                "custom_config",
                default_value=default_config_path,
                description="Path to config file",
            ),
            # 👇 NEW ARGUMENT
            DeclareLaunchArgument(
                "pcl_topic",
                default_value="/uav2/losos_server/current_submap_pc",
                description="Input point cloud topic",
            ),
            # this adds the args to the list of args available for this launch files
            # these args can be listed at runtime using -s flag
            # default_value is required to if the arg is supposed to be optional at launch time
            DeclareLaunchArgument(
                name="debug",
                default_value="false",
                description="Runs the node within a gdb debug session.",
            ),
        ]
    )

    debug = IfElseSubstitution(
        condition=PythonExpression(['"', debug, '" == "true"']),
        if_value="debug_roslaunch " + os.ttyname(sys.stdout.fileno()),
        else_value="",
    )

    # -------------------------------------------------
    # Node definition
    # -------------------------------------------------
    rbl_controller_node = ComposableNode(
        package=pkg_name,
        plugin="rbl_controller::WrapperRosRBL",
        name="rbl_controller",
        namespace=uav_name,
        parameters=[
            custom_config,
            {"config": custom_config},
            {"custom_config": custom_config},
            {"simulation": True},
            {"uav_name": uav_name},
            {"control_frame": [uav_name, "/world_origin"]},
        ],
        remappings=[
            ("~/odom_in", "estimation_manager/odom_main"),
            ("~/alt_in", "estimation_manager/garmin_agl/agl_height"),
            # 👇 NOW CONFIGURABLE
            ("~/pcl_in", pcl_topic),
            ("~/octomap_in", "octomap_server/octomap_local_binary"),
            ("~/group_states_in", "filter_reflective_uavs/pose_vel"),
            ("~/tracker_cmd_in", "control_manager/tracker_cmd"),
            ("~/ref_out", "control_manager/reference"),
            ("~/goto_out", "~/goto"),
            ("~/control_activation_in", "~/activation"),
            ("~/control_activation_params_in", "~/activation_params"),
            ("~/control_deactivation_in", "~/deactivation"),
            ("~/fly_to_start_in", "~/fly_to_start"),
            ("~/destination_out", "~/destination_vis"),
            ("~/centroid_out", "~/centroid_vis"),
            ("~/position_out", "~/position_vis"),
            ("~/obstacle_markers_out", "~/obstacles_vis"),
            ("~/neighbors_markers_out", "~/neighbors_vis"),
            ("~/hull_markers_out", "~/hull_vis"),
            ("~/line_out", "~/line_vis"),
        ],
    )

    # -------------------------------------------------
    # External container
    # -------------------------------------------------
    ld.add_action(
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[rbl_controller_node],
            condition=UnlessCondition(standalone),
        )
    )

    # -------------------------------------------------
    # Standalone container
    # -------------------------------------------------
    ld.add_action(
        ComposableNodeContainer(
            namespace=uav_name,
            name="rbl_controller_container",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[rbl_controller_node],
            output="screen",
            prefix=[debug],
            condition=IfCondition(standalone),
        )
    )

    return ld
