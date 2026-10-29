from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    model_name = LaunchConfiguration("model").perform(context)

    moveit_config = (
        MoveItConfigsBuilder(model_name, package_name="container_unloading_sim_benchmark")
        .robot_description_kinematics("urdf/moveit/kinematics.yaml")
        .robot_description_semantic(f"urdf/moveit/{model_name}.srdf")
        .pilz_cartesian_limits("urdf/moveit/pilz_cartesian_limits.yaml")
        .trajectory_execution("urdf/moveit/moveit_controllers.yaml")
        .joint_limits("config/ur30/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="stomp",
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "trajectory_execution.allowed_start_tolerance": 0.05,
                "trajectory_execution.execution_duration_monitoring": False,
            },
        ],
    )

    return [move_group_node]


def generate_launch_description():
    model = DeclareLaunchArgument(
        "model",
        default_value="emrox_ur30_d435_epick",
        description="URDF/XACRO model name (without file extension)",
    )

    return LaunchDescription([model, OpaqueFunction(function=launch_setup)])
