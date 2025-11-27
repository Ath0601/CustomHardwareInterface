import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_dir = get_package_share_directory("description")
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(desc_dir, "urdf", "panda_robot.urdf.xacro")
    bringup_dir = get_package_share_directory("bringup")
    controllers_yaml = os.path.join(bringup_dir, "config", "controller.yaml")
    moveit_dir = get_package_share_directory("moveit_config")
    rviz_dir = os.path.join(get_package_share_directory("bringup"),"rvizenv.rviz")

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf,
    ])

    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[' -d', rviz_dir],
        output='screen'
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {'robot_description': robot_description_param},
            controllers_yaml,
        ],
        output="screen"
    )

    spawner_js = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawner_robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_dir, 'launch', 'move_group.launch.py'])
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        ros2_control_node,
        spawner_js,
        spawner_robot_controller,
        rviz_node,
        moveit_launch
    ])
