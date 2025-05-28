import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    path_to_sdf_model = os.path.join(get_package_share_directory('drone_navigation'), "models/drone_model.sdf")

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=path_to_sdf_model,
        description='Full path to the SDF world file'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('world'),
        }.items()
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/world/quadcopter/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # Delay starting the bridge to ensure Gazebo topic is ready
    gz_bridge_node_delayed = TimerAction(
        period=5.0,
        actions=[gz_bridge_node]
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gz_sim_launch)
    ld.add_action(gz_bridge_node_delayed)

    return ld