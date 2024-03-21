from os import environ
import netifaces as ni
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals, IfCondition

ip = ni.ifaddresses('mlan0')[ni.AF_INET][0]['addr']
print('mlan0: {:s}:4242'.format(ip))

ARGUMENTS = [ DeclareLaunchArgument('cam', default_value='true')]

def generate_launch_description():

    foxglove_websockets = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])]),
            launch_arguments=[('address','{:s}'.format(ip)),
                            ('capabilities','[clientPublish,services,connectionGraph,assets]'),
                            ('topic_whitelist', "/calibration_camera_feed"),
                            ('service_whitelist', '[""]'),
                            ('param_whitelist', '[""]'),
                            ('send_buffer_limit','10000000'),
                            ('num_threads', '0'),
                            ('use_compression', 'false'),
                            ('use_sim_time', 'false')])
    
    cam = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('b3rb_bringup'), 'launch', 'ov5645.launch.xml'])]),
        condition=IfCondition(LaunchConfiguration('cam')))
    
    b3rb_camera_calibration = Node(
            package='b3rb_camera_calibration',
            executable='cameracalibrator',
            name='b3rb_camera_calibration',
            output='screen',
            parameters=[{'size': '6x9',
                         'square': 0.02,
                         'camera': '/my_camera'}],
            remappings=[('/image', '/camera/image_raw')],
        )
    
    return LaunchDescription(ARGUMENTS + [
        b3rb_camera_calibration,
        foxglove_websockets,
        cam,
    ])

