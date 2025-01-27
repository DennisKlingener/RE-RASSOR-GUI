"""Launch Gazebo with the specified world file (empty world by default)"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.opaque_function import OpaqueFunction
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
import os
from launch.actions import TimerAction

def __spawn_robot(context, *args, **kwargs):
    """Returns the nodes for spawning a unique robot in Gazebo."""

    pkg_ezrassor_sim_description = os.path.join(
        get_package_share_directory("ezrassor_sim_description")
    )

    robot_name = LaunchConfiguration("robot_name").perform(context)
    if robot_name[0] == "/":
        robot_name = robot_name[1:]

    xacro_file = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "ezrassor.xacro.urdf"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=",
            robot_name,
        ]
    )
    params = {"robot_description": robot_description_content}

    namespace = robot_name
    if namespace[0] != "/":
        namespace = f"/{namespace}"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[params],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-entity",
            robot_name,
            "-robot_namespace",
            namespace,
            "-topic",
            f"{namespace}/robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "-c",
            f"/{robot_name}/controller_manager",
            "--set-state",
            "active",
            "diff_drive_controller",
        ],
        output="screen",
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        robot_state_publisher,
        spawn_entity,
    ]

def __spawn_paver(context, *args, **kwargs):
    """Returns the nodes for spawning a paver in Gazebo."""

    pkg_ezrassor_sim_description = os.path.join(
        get_package_share_directory("ezrassor_sim_description")
    )

    xacro_file = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "paver.xacro"
    )

    paver_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    params = {"robot_description": paver_description_content}

    paver_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="paver",
        output="screen",
        parameters=[params],
    )

    spawn_paver = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace="/paver",
        arguments=[
            "-entity", "paver",
            "-topic", "/paver/robot_description",
            "-x", LaunchConfiguration("paver_x"),
            "-y", LaunchConfiguration("paver_y"),
            "-z", LaunchConfiguration("paver_z"),
        ],
        output="screen",
    )

    return [paver_state_publisher, spawn_paver]

def __spawn_paver2(context, *args, **kwargs):
    """Returns the nodes for spawning a second paver in Gazebo."""

    pkg_ezrassor_sim_description = os.path.join(
        get_package_share_directory("ezrassor_sim_description")
    )

    xacro_file = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "paver.xacro"
    )

    paver_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    params = {"robot_description": paver_description_content}

    paver_state_publisher2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="paver2",
        output="screen",
        parameters=[params],
    )

    spawn_paver2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace="/paver2",
        arguments=[
            "-entity", "paver2",
            "-topic", "/paver2/robot_description",
            "-x", LaunchConfiguration("paver_x"),
            "-y", LaunchConfiguration("paver_y"),
            "-z", LaunchConfiguration("paver_z2"),  # Adjusted z position for stacking
        ],
        output="screen",
    )

    return [paver_state_publisher2, spawn_paver2]

def __spawn_paver3(context, *args, **kwargs):
    """Returns the nodes for spawning a third paver in Gazebo."""

    pkg_ezrassor_sim_description = os.path.join(
        get_package_share_directory("ezrassor_sim_description")
    )

    xacro_file = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "paver.xacro"
    )

    paver_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    params = {"robot_description": paver_description_content}

    paver_state_publisher3 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="paver3",
        output="screen",
        parameters=[params],
    )

    spawn_paver3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace="/paver3",
        arguments=[
            "-entity", "paver3",
            "-topic", "/paver3/robot_description",
            "-x", LaunchConfiguration("paver_x"),
            "-y", LaunchConfiguration("paver_y"),
            "-z", LaunchConfiguration("paver_z3"),  # Adjusted z position for stacking
        ],
        output="screen",
    )

    return [paver_state_publisher3, spawn_paver3]


def generate_launch_description():
    """Spawn a new instance of Gazebo Classic with an optional world file."""

    robot_name_argument = DeclareLaunchArgument(
        "robot_name",
        default_value="ezrassor",
        description="Entity name and namespace for robot spawn (default: ezrassor)",
    )

    x_position_argument = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="X position for robot spawn: [float]",
    )
    y_position_argument = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Y position for robot spawn: [float]",
    )
    z_position_argument = DeclareLaunchArgument(
        "z",
        default_value="0.2",
        description="Z position for robot spawn: [float]",
    )
    r_axis_argument = DeclareLaunchArgument(
        "R",
        default_value="0.0",
        description="Roll angle for robot spawn: [float]",
    )
    p_axis_argument = DeclareLaunchArgument(
        "P",
        default_value="0.0",
        description="Pitch angle for robot spawn: [float]",
    )
    y_axis_argument = DeclareLaunchArgument(
        "Y",
        default_value="0.0",
        description="Yaw angle for robot spawn: [float]",
    )

    wheels_driver_node = Node(
        package="ezrassor_sim_description",
        executable="wheels_driver",
        namespace=LaunchConfiguration("robot_name"),
        output={"both": "screen"},
    )

    paver1_x_position_argument = DeclareLaunchArgument(
        "paver_x",
        default_value="-0.8",
        description="X position for paver spawn: [float]",
    )
    paver1_y_position_argument = DeclareLaunchArgument(
        "paver_y",
        default_value="0.0",
        description="Y position for paver spawn: [float]",
    )
    paver1_z_position_argument = DeclareLaunchArgument(
        "paver_z",
        default_value="3.0",
        description="Z position for paver spawn: [float]",
    )

    paver2_z_position_argument = DeclareLaunchArgument(
        "paver_z2",
        default_value="4.0",  # Adjust the height for the second paver
        description="Z position for second paver spawn: [float]",
    )

    paver3_z_position_argument = DeclareLaunchArgument(
        "paver_z3",
        default_value="4.5",  # Adjust the height for the second paver
        description="Z position for third paver spawn: [float]",
    )

    paver1_delay = TimerAction(period=0.5, actions=[OpaqueFunction(function=__spawn_paver)])  # 15 seconds delay
    paver2_delay = TimerAction(period=1.0, actions=[OpaqueFunction(function=__spawn_paver2)])  # 15 seconds delay
    paver3_delay = TimerAction(period=1.5, actions=[OpaqueFunction(function=__spawn_paver3)])  # 15 seconds delay

    return LaunchDescription(
        [
            robot_name_argument,
            x_position_argument,
            y_position_argument,
            z_position_argument,
            r_axis_argument,
            p_axis_argument,
            y_axis_argument,
            OpaqueFunction(function=__spawn_robot),
            wheels_driver_node,
            paver1_x_position_argument,
            paver1_y_position_argument,
            paver1_z_position_argument,
            paver2_z_position_argument,
            paver3_z_position_argument,
            paver1_delay,
            paver2_delay,
            paver3_delay, 
        ]
    )
