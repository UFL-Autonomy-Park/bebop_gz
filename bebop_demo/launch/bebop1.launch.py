from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package dirs
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('bebop_demo')

    # Bebop names and initial conditions as JSON strings
    robot_names = '["bebop1"]'  
    initial_conditions = '[[2.5, -0.5, 0.0, 1.0]]' 
    lider = 'bebop1'

    State = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_gui', 'bebop_gui_one',],
        output='screen'
    )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -z  1000000 bebop1.sdf'
        }.items(),
    )
    # Launch ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={os.path.join(pkg_ros_gz_sim_demos, "config", "bebop1.yaml")}'
        ],
        output='screen'
    )

    # Launch setpoint node
    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args',
            '-p', 'h:=0.3',
            '-p', 'r:=0.2',
            '-p', f'lider_name:={lider}',
        ],
        output='screen'
    )


    # Launch multi-robot pose publisher to set initial positions
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose', 
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': robot_names},
            {'initial_conditions': initial_conditions}
        ]
    )

    # Launch PID controller 
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': f'{lider}',
            'goal_name': 'goal'
        }.items()
    )

    return LaunchDescription([
        State,
        gz_sim,
        ros_gz_bridge,
        setpoint,
        set_pose,
        controller,
    ])