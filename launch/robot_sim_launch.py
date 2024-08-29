import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import FindExecutable



def generate_launch_description():

    package_name='lanealucys_robot'
    
    nav = LaunchConfiguration('nav2')
    slam = LaunchConfiguration('slam')
    rviz_gazebo = LaunchConfiguration('rviz_gazebo')
    rviz_nav2 = LaunchConfiguration('rviz_nav2')

    robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_launch.py')]), 
                launch_arguments={'sim': 'true', 'nav2': nav, 'rviz_nav2': rviz_nav2, 'slam': slam, 'namespace': ''}.items()
            )

    # Include the Gazebo wildthumper_playpen launch file, provided by the ardupilot_gz_bringup package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'wildthumper_playpen.launch.py')]),
                launch_arguments={'use_gz_tf': 'false', 'rviz': rviz_gazebo}.items()
            )
    
    #mavros = IncludeLaunchDescription(
    #            XMLLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory('mavros'),'launch','apm.launch'
    #            )]), launch_arguments={'respawn_mavros': 'true','fcu_url': 'udp://127.0.0.1:14550@', 'gcs_url': 'udp://@127.0.0.1:14552'}.items()
    #)
    
    mavros = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'mavros_launch.py')]),
                launch_arguments={'use_sim_time': 'true', 'respawn_mavros': 'true', 'fcu_url': 'udp://127.0.0.1:14550@', 'gcs_url': 'udp://@127.0.0.1:14552', 'pluginlists_yaml': [os.path.join(
                    get_package_share_directory(package_name), 'config', 'mavros_pluginlists.yaml')], 'mavros_pluginlists.yaml': [os.path.join(
                    get_package_share_directory(package_name), 'config', 'mavros_config.yaml')]}.items()
            )
    
    
    odom_waiter = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic echo ',
            '/mavros/local_position/odom',
            ' nav_msgs/msg/Odometry',
            ' --once'
        ]],
        shell=True
    )
    
    micro_ros_agent = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            parameters=[{"use_sim_time": 'true'}],
            output="both",
            arguments=['udp4', '--port', '2019'],
    )



    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description='start nav2'),
        DeclareLaunchArgument(
            'slam',
            choices=[
                'slam_toolbox',
                'cartographer',
                'none'
            ],
            default_value='slam_toolbox',
            description='which slam to use'),
        DeclareLaunchArgument(
            'rviz_gazebo',
            default_value='false',
            description='start gazebo rviz'),
        DeclareLaunchArgument(
            'rviz_nav2',
            default_value='true',
            description='start nav2 rviz'),
        gazebo,
        #mavros,
        #RegisterEventHandler(
        #    OnExecutionComplete(
        #        target_action=odom_waiter,
        #        on_completion=[
        #            LogInfo(msg='Received first odom message...'),
        #            robot
        #        ]
        #    )
        #),
        #odom_waiter,
        micro_ros_agent,
        #robot
    ])


