import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
import yaml


def launch_setup(context, *args, **kwargs):

    # Gazebo world
    world_model = LaunchConfiguration("world_model").perform(context)

    world_path = os.path.join(
        get_package_share_directory("container_unloading_sim_benchmark"),
        "world",
        f"{world_model}.sdf",
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution(
            [
                get_package_share_directory("container_unloading_sim_benchmark"),
                "world",
                "models",
            ]
        ),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    # Robot
    robot_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join(
            [
                os.path.dirname(get_package_share_directory("rox_description")),
                os.path.dirname(get_package_share_directory("epick_description")),
                os.path.dirname(get_package_share_directory("realsense2_description")),
            ]
        ),
    )

    bridge_config_file = os.path.join(
        get_package_share_directory("container_unloading_sim_benchmark"),
        "config",
        "gz_bridge_config.yaml",
    )

    x, y, z, R, P, Y = "0.0 0.0 0.0 0.0 0.0 0.0".split()

    arm_pos_path = os.path.join(
        get_package_share_directory("container_unloading_sim_benchmark"),
        "config",
        "ur30",
        LaunchConfiguration("initial_arm_pos").perform(context) + ".yaml",
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("container_unloading_sim_benchmark"),
                "launch",
                "robot_description_bringup.launch.py",
            )
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "initial_arm_pos": arm_pos_path,
            "use_gazebo": "True",
        }.items(),
    )

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_model",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            LaunchConfiguration("model"),
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            R,
            "-P",
            P,
            "-Y",
            Y,
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config_file}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    set_sim_time_arg = SetParameter(name="use_sim_time", value=True)

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("container_unloading_sim_benchmark"),
                "urdf",
                "moveit",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"model": LaunchConfiguration("model")}.items(),
        condition=IfCondition(LaunchConfiguration("moveit"))
    )

    pkg_share = get_package_share_directory("container_unloading_sim_benchmark")

    def load_yaml(*path_parts):
        with open(os.path.join(pkg_share, *path_parts)) as f:
            return yaml.safe_load(f)

    use_moveit = LaunchConfiguration("moveit").perform(context).lower() == "true"

    rviz_parameters = []

    if use_moveit:
        rviz_parameters.append(
            {
                "robot_description_kinematics": load_yaml(
                    "urdf", "moveit", "kinematics.yaml"
                ),
                "robot_description_planning": load_yaml(
                    "config", "ur30", "joint_limits.yaml"
                ),
            }
        )

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("container_unloading_sim_benchmark"),
                    "config",
                    "sim_config.rviz",
                ),
            ],
            parameters=rviz_parameters,
        )

    metrics_extractor = Node(
        package="container_unloading_sim_benchmark",
        executable="extract_metrics.py",  
        name="metrics_extractor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_dir": "~/unloading_reports",
            }
        ],
    )

    return [
        set_sim_time_arg,
        set_gz_resource_path,
        robot_gz_resource_path,
        gazebo_launch,
        robot_description_launch,
        gz_spawn,
        gz_bridge,
        controller_manager,
        rviz_node,
        moveit_launch,
        metrics_extractor,
    ]


def generate_launch_description():
    world_model = DeclareLaunchArgument(
        "world_model",
        default_value="container_unloading",
        description="World model name (without file extension).",
    )

    model = DeclareLaunchArgument(
        "model",
        default_value="emrox_ur30_d435_epick",
        description="URDF/XACRO model name (without file extension)",
    )

    initial_arm_pos = DeclareLaunchArgument(
        "initial_arm_pos",
        default_value="gz_initial_positions",
        description="Initial joint positions file for the arm",
    )

    moveit = DeclareLaunchArgument(
        "moveit",
        default_value="false",
        description="Whether to launch MoveIt or not",
    )

    return LaunchDescription(
        [world_model, model, initial_arm_pos, moveit, OpaqueFunction(function=launch_setup)]
    )
