import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    carter_description = get_package_share_directory("carter_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(carter_description, "urdf", "carter.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(carter_description).parent.resolve())],
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "carter",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.22",
            "-r",
            "0",
            "-p",
            "0",
            "-Y",
            "0",
            "-topic",
            "/robot_description",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        ],
        output="screen",
        remappings=[
            ("/model/diff_drive/odometry", "/odom"),
            ("/odom/tf", "/tf"),
        ],
    )

    rviz_file = "rviz.rviz"
    rviz_path = PathJoinSubstitution(
        [get_package_share_directory("carter_description"), rviz_file]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )

    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
            rviz2_node,
        ]
    )
