import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    robot_controller = LaunchConfiguration("robot_controller")
    controllers_file = LaunchConfiguration("controllers_file")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    start_rviz = LaunchConfiguration("start_rviz")
    paused = LaunchConfiguration("paused")
    model_check = LaunchConfiguration("model_check")

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_type",
            default_value="Rizon4s",
            description="Single-arm Flexiv robot type (config folder in flexiv_description).",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="thunder_computed_torque_controller",
            description="Controller to start, as named in the controllers file.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("flexiv_gazebo"), "config", "flexiv_gazebo_controllers.yaml"]
            ),
            description="ros2_control controllers configuration.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("flexiv_gazebo"), "worlds", "empty.world"]
            ),
            description="Gazebo world file.",
        ),
        DeclareLaunchArgument("gui", default_value="true", description="Start the Gazebo GUI."),
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Pause Gazebo again as soon as the controllers are active, to step manually.",
        ),
        DeclareLaunchArgument("start_rviz", default_value="false", description="Start RViz."),
        DeclareLaunchArgument(
            "model_check",
            default_value="true",
            description="Compare Thunder inverse dynamics with the joint torque sensors.",
        ),
    ]

    # Let Gazebo resolve package://flexiv_description/... mesh URIs, keeping its built-in models
    description_share_parent = os.path.dirname(get_package_share_directory("flexiv_description"))
    gazebo_model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", description_share_parent + ":/usr/share/gazebo-11/models"
    )
    # Never try to download models from the online database (blocks gzserver startup)
    gazebo_model_database = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # The trajectory controller commands positions, the torque controllers efforts: the joints
    # expose only the interface the chosen controller needs (see flexiv_gazebo.urdf.xacro)
    command_interface = PythonExpression(
        ["'position_pid' if '", robot_controller, "' == 'rizon_arm_controller' else 'effort'"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("flexiv_gazebo"), "urdf", "flexiv_gazebo.urdf.xacro"]
                ),
                " robot_type:=",
                robot_type,
                " controllers_file:=",
                controllers_file,
                " command_interface:=",
                command_interface,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world, "gui": gui, "pause": "true"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "flexiv"],
        output="screen",
    )

    # Both controllers in one switch: they become active together, on the first physics step
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            robot_controller,
            "--controller-manager",
            "/controller_manager",
            "--activate-as-group",
            "--switch-timeout",
            "60",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("flexiv_description"), "rviz", "view_flexiv.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(start_rviz),
    )

    # Thunder model vs Gazebo joint torque sensors (damping/friction as in the URDF <dynamics>)
    thunder_model_check = Node(
        package="thunder_controllers",
        executable="thunder_model_check",
        parameters=[
            {
                "use_sim_time": True,
                "thunder_par_file": "Rizon4s_par.yaml",
                "damping": [1.0] * 7,
                "friction": [0.0] * 7,
            }
        ],
        output="screen",
        condition=IfCondition(model_check),
    )

    # Controllers are activated inside the update loop: unpause once the spawner has requested
    # the switch, so that the arm never runs without torque (it stays at the spawn pose)
    unpause_physics = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty"],
                output="screen",
            )
        ],
    )

    # Optionally pause again right after activation, to step manually
    pause_physics = ExecuteProcess(
        cmd=["ros2", "service", "call", "/pause_physics", "std_srvs/srv/Empty"],
        output="screen",
        condition=IfCondition(paused),
    )

    # Start controllers only once the robot is spawned, and in order
    delayed_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[controllers_spawner, unpause_physics])
    )
    delayed_pause_and_check = RegisterEventHandler(
        OnProcessExit(
            target_action=controllers_spawner, on_exit=[thunder_model_check, pause_physics]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_model_path,
            gazebo_model_database,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            delayed_controllers,
            delayed_pause_and_check,
            rviz,
        ]
    )
