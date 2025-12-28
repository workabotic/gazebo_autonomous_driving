from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

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

    # Include the Ackermann vehicle launch file
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


    # Include the neural network autopilot launch file
    autopilot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('autopilot_neural_network'),
                                  'launch',
                                  'autopilot.launch.py'])))

    # Create and return the launch description
    launch_description = LaunchDescription([world_arg, x_arg, y_arg, z_arg, 
                                            roll_arg, pitch_arg, yaw_arg, 
                                            vehicle_launch, autopilot_launch])

    return launch_description
