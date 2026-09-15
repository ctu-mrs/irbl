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
    use_sim_time = LaunchConfiguration("use_sim_time")
    control_frame = LaunchConfiguration("control_frame")

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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable(
                    "USE_SIM_TIME", default_value="false"
                ),
            ),
            # Frame all UAVs plan and avoid each other in. Left empty (the default), it is
            # derived per-UAV as "<uav_name>/world_origin" (single-UAV behavior). For a multi-UAV
            # simulation, every UAV must share the *same* world frame (matching the one obstacle
            # map/frame the map_generator node publishes in), so pass this explicitly there.
            DeclareLaunchArgument(
                "control_frame",
                default_value="",
                description="Shared world frame; defaults to '<uav_name>/world_origin' if empty",
            ),
        ]
    )

    debug = IfElseSubstitution(
        condition=PythonExpression(['"', debug, '" == "true"']),
        if_value="debug_roslaunch " + os.ttyname(sys.stdout.fileno()),
        else_value="",
    )

    control_frame = IfElseSubstitution(
        condition=PythonExpression(['"', control_frame, '" == ""']),
        if_value=[uav_name, "/world_origin"],
        else_value=control_frame,
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
            {"use_sim_time": use_sim_time},
            {"uav_name": uav_name},
            {"control_frame": control_frame},
        ],
        remappings=[
            ("~/odom_in", "odometry"),
            ("~/alt_in", "alt"),
            # 👇 NOW CONFIGURABLE
            ("~/pcl_in", pcl_topic),
            ("~/octomap_in", "octomap_server/octomap_local_binary"),
            ("~/group_states_in", "filter_reflective_uavs/pose_vel"),
            ("~/sim_group_poses_in", "/multirotor_simulator/uav_poses"),
            ("~/ref_out", "reference"),
            ("~/goto_out", "~/goto"),
            ("~/control_activation_in", "~/activation"),
            ("~/control_deactivation_in", "~/deactivation"),
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
