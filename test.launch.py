import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_token = os.getenv('ROBOT_TOKEN','')
    robot_ip = os.getenv('ROBOT_IP')
    on_exit = LaunchConfiguration('on_exit', default='shutdown')
    elevenlabs_api_key = LaunchConfiguration(
        'elevenlabs_api_key', default=EnvironmentVariable(
            'ELEVENLABS_API_KEY', default_value=''))
    voice_name = LaunchConfiguration('voice_name', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value=os.getenv('ROBOT_IP', os.getenv('GO2_IP', '')),
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'on_exit',
            default_value='shutdown',
            description='Behavior when a node exits (shutdown will terminate all nodes)'
        ),
        DeclareLaunchArgument(
            'elevenlabs_api_key',
            default_value=EnvironmentVariable('ELEVENLABS_API_KEY', default_value='YOUR_ELEVENLABS_API_KEY'),
            description='API key for ElevenLabs TTS service'
        ),
        DeclareLaunchArgument(
            'voice_name',
            default_value='flHkNRp1BlvT73UL6gyz', # default voice
            description='Voice name for TTS'
        ),
        Node(
            package='go2_robot_sdk',
            executable='paws_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token}],
            ),
        Node(
            package='go2_robot_sdk',
            executable='paws_go2_control_node',
            ),
        Node(
            package='go2_robot_sdk',
            executable='tts_node',
            name='tts_node',
            parameters=[{
                'elevenlabs_api_key': elevenlabs_api_key,
                'voice_name': voice_name
            }],
            on_exit=on_exit,
            ),
    ])