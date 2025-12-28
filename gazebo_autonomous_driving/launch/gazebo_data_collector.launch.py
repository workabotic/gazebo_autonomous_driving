from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription,
                            ExecuteProcess,
                            RegisterEventHandler)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default world file (used if not overridden from the command line)
    default_world = PathJoinSubstitution([FindPackageShare('gazebo_racing_tracks'),
                                          'worlds',
                                          'grass_track.sdf'])

    # Declare launch arguments for world selection
    world_arg = DeclareLaunchArgument('world', default_value=default_world,
                                      description='Path to the Gazebo world file')

    # Declare launch arguments for initial vehicle pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial Y position')

    z_arg = DeclareLaunchArgument('z', default_value='0.0',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll (rad)')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch (rad)')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw (rad)')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Launch Gazebo + vehicle
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ackermann_steering_vehicle'),
                                  'launch',
                                  'vehicle.launch.py'])),
        launch_arguments={'world': world_file,
                          'x': x,
                          'y': y,
                          'z': z,
                          'R': roll,
                          'P': pitch,
                          'Y': yaw}.items())

    # Launch joystick teleoperation
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ackermann_steering_vehicle'),
                                  'launch',
                                  'joystick.launch.py'])))

    # Blocking process: wait until /clock publishes once
    wait_for_clock = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/clock', '--once'],
        output='screen'
    )

    # Data collector (starts ONLY after /clock is available)
    data_collector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('autopilot_neural_network'),
                                  'launch',
                                  'data_collector.launch.py']))
    )

    # Event handler: start data collector after /clock appears
    start_data_collector_on_clock = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_clock,
            on_exit=[data_collector_launch]
        )
    )

    # Create and return the launch description
    launch_description = LaunchDescription([world_arg, 
                                            x_arg, 
                                            y_arg, 
                                            z_arg, 
                                            roll_arg, 
                                            pitch_arg, 
                                            yaw_arg, 
                                            vehicle_launch, 
                                            joystick_launch, 
                                            wait_for_clock, 
                                            start_data_collector_on_clock])

    return launch_description
