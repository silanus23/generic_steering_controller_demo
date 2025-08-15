from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="false",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    # Get URDF via xacro - use your own URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("carlikebot_bringup"), "urdf", "carlikebot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Use your controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("carlikebot_bringup"),
            "config",
            "carlikebot_controllers.yaml",
        ]
    )
    
    # Use the carlikebot rviz config from the demo description
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("carlikebot_bringup"),
            "rviz",
            "carlikebot.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],  # Add robot_description
        output="both",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Use your custom controller with optional odometry remapping
    carlikebot_controller_spawner_remapped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "carlikebot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /carlikebot_base_controller/tf_odometry:=/tf",
        ],
        condition=IfCondition(remap_odometry_tf),
    )

    carlikebot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["carlikebot_base_controller", "--param-file", robot_controllers],
        condition=UnlessCondition(remap_odometry_tf),
    )
    delay_controller_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[carlikebot_controller_spawner, carlikebot_controller_spawner_remapped],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after controller spawning
    delay_joint_state_broadcaster_after_controller_spawner_remapped = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=carlikebot_controller_spawner_remapped,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_joint_state_broadcaster_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=carlikebot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        carlikebot_controller_spawner_remapped,
        carlikebot_controller_spawner,
        delay_controller_spawner_after_control_node,
        delay_joint_state_broadcaster_after_controller_spawner_remapped,
        delay_joint_state_broadcaster_after_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)