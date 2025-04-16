from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial',
            default_value='/dev/ttyUSB0',
            description='UART port of the Pozyx'
        ),
        DeclareLaunchArgument(
            'targetDeviceId',
            default_value='0x1234',
            description='Target device ID of the Pozyx'
        ),
        DeclareLaunchArgument(
            'debug_level',
            default_value='0',
            description='Log debug level.'
        ),
        Node(
            package='ros2_uwb_tools',
            executable='pozyx_ranging_reader',
            name='pozyx_ranging_reader',
            output='screen',
            parameters=[
                {'targetDeviceId': LaunchConfiguration('targetDeviceId')},
                {'serial': LaunchConfiguration('serial')},
                {'debug_level': LaunchConfiguration('debug_level')}
            ]
        ),
    ]) 