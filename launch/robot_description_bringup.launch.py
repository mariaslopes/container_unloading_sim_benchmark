import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # get the model parameter value
    model_name = LaunchConfiguration("model").perform(context)
    urdf = os.path.join(
        get_package_share_directory("container_unloading_sim_benchmark"),
        "urdf",
        f"{model_name}.xacro",
    )

    use_gazebo = LaunchConfiguration("use_gazebo").perform(context)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro",
                            " ",
                            urdf,
                            " ",
                            "initial_positions:=",
                            LaunchConfiguration("initial_arm_pos"),
                            " ",
                            "gazebo:=",
                            use_gazebo
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
    )

    # return the nodes that should be launched
    return [rsp]


def generate_launch_description():
    model = DeclareLaunchArgument(
        "model",
        default_value="emrox_ur30_d435_epick",
        description="URDF/XACRO model name (without file extension)",
    )

    use_gazebo = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Whether to use Gazebo for simulation",
    )

    initial_arm_pos = DeclareLaunchArgument(
        "initial_arm_pos",
        default_value="",
        description="Initial position of the robot arm joints (6 values space separated)",
    )

    # Use OpaqueFunction to execute code that requires the context
    return LaunchDescription(
        [model,use_gazebo, initial_arm_pos, OpaqueFunction(function=launch_setup)]
    )
