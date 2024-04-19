import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robots'] = crazyflies['robots']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robot_types'] = crazyflies['robot_types']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['all'] = crazyflies['all']

    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:

        robot_desc = f.read()
    server_yaml_content["/crazyflie_server"]["ros__parameters"]["robot_description"] = robot_desc

    # Save server and mocap in temp file such that nodes can read it out later
    with open('tmp_server.yaml', 'w') as outfile:
        yaml.dump(server_yaml_content, outfile, default_flow_style=False, sort_keys=False)


    crazyflie_name = '/cf231'


    return LaunchDescription([
        DeclareLaunchArgument('server_yaml_file', default_value=''),
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
        ),
        Node(
            package='crazyflie',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.3},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': crazyflie_name}]
        ),
        Node(
            package='crazyflie',
            executable='simple_mapper_multiranger.py',
            name='simple_mapper_multiranger',
            output='screen',
            parameters=[
                        {'robot_prefix': crazyflie_name}]
        ),
    ])
