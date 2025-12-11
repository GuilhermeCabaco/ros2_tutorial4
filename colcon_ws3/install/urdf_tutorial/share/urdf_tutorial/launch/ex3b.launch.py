from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    arm_prefix = LaunchConfiguration("prefix", default="")
    arm_model = LaunchConfiguration("ur_type", default="ur3")
    desc_pkg = LaunchConfiguration("description_package", default="urdf_tutorial")
    desc_file = LaunchConfiguration("description_file", default="ur.urdf.xacro")
    open_rviz = LaunchConfiguration("launch_rviz", default="true")

    robot_description_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(desc_pkg), "urdf", desc_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            arm_model,
            " ",
            "prefix:=",
            arm_prefix,
        ]
    )
    robot_description_param = {"robot_description": robot_description_cmd}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description_param],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        on_exit=Shutdown(),
    )

    log_robot_description = LogInfo(
        msg=robot_description_param.get("robot_description"),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rviz2_marker_demo"), "rviz", "rviz2_marker_demo.rviz"]
    )

    rviz2_marker_demo = Node(
        package="rviz2_marker_demo",
        executable="rviz_node",
        name="rviz_node",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(open_rviz),
        on_exit=Shutdown(),
    )


    # world -> assembly_frame 
    world_to_assembly = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_assembly",
        #            x     y    z   qx   qy   qz   qw    parent   child
        arguments=["0.3", "0", "0", "0", "0", "0", "1", "world", "assembly_frame"],
    )

    # world -> base_link
    ur3_base_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ur3_base_broadcaster",
        #            x     y    z   qx   qy   qz   qw    parent   child
        arguments=["0.3", "0", "0", "0", "0", "0", "1", "world", "base_link"],
    )

    # tf_11 -> grasp_11
    grasp_11_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="grasp_11_frame",
        #           x    y     z     yaw pitch   roll     parent    child
        arguments=["0", "0", "0.15", "0", "0", "3.1416", "tf_11", "grasp_11"],
    
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
        rviz2_marker_demo,
        world_to_assembly,
        ur3_base_broadcaster,
        grasp_11_frame,
        log_robot_description,
    ]

    return nodes_to_start


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        name="launch_rviz",
        default_value="true",
        description="Launch RViz2 automatically?",
    )

    return LaunchDescription(
        [
            rviz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
