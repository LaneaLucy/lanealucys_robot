import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    package_name='lanealucys_robot'
    
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')
    namespace = LaunchConfiguration('namespace')
    pluginlists_yaml = LaunchConfiguration('pluginlists_yaml')
    config_yaml = LaunchConfiguration('config_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    mavros = Node(
            package='mavros',
            namespace=namespace,
            executable='mavros_node',
            output='screen',
            respawn=respawn_mavros,
            parameters=[{'use_sim_time': use_sim_time, 'fcu_url': fcu_url, 'gcs_url': gcs_url, 'tgt_system': tgt_system, 'tgt_component': tgt_component, 'fcu_protocol': fcu_protocol}, pluginlists_yaml, config_yaml]
    )




    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='fcu_url'),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='gcs_url'),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='tgt_system'),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='tgt_component'),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='log_output'),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='fcu_protocol'),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='respawn_mavros'),
        DeclareLaunchArgument(
            'namespace',
            default_value='mavros',
            description='namespace'),
        DeclareLaunchArgument(
            'pluginlists_yaml',
            default_value=os.path.join(get_package_share_directory('mavros'), 'launch', 'apm_pluginlists.yaml'),
            description='pluginlists_yaml'),
        DeclareLaunchArgument(
            'config_yaml',
            default_value=os.path.join(get_package_share_directory('mavros'), 'launch', 'apm_config.yaml'),
            description='config_yaml'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        mavros
    ])
