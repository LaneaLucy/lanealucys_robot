import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetRemap

from launch.conditions import IfCondition, UnlessCondition

from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit



def generate_launch_description():

    package_name='lanealucys_robot'
    
    nav = LaunchConfiguration('nav2')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_toolbox = LaunchConfiguration('slam_toolbox')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('sim')
    rviz_nav2 = LaunchConfiguration('rviz_nav2')

    lidar = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld14p',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14P'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate' : 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  # unit is degress
        {'angle_crop_max': 225.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 8.0},   # unit is meter
        {'--ros-args': '--params-file /root/ros_ws/example_qos_overrides_with_wildcard.yaml'}
      ]
    )

    mavros_old = IncludeLaunchDescription(
                XMLLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('mavros'),'launch','apm.launch'
                )]), launch_arguments={'fcu_url': 'serial:///dev/ttyACM0', 'gcs_url': 'udp-b://@'}.items()
    )
    mavros = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'mavros_launch.py')]),
                launch_arguments={
                    'use_sim_time': 'true', 
                    'respawn_mavros': 'true', 
                    'fcu_url': 'serial:///dev/ttyACM0', 
                    'gcs_url': 'udp-b://@', 
                    'pluginlists_yaml': [os.path.join(get_package_share_directory(package_name), 'config', 'mavros_pluginlists.yaml')], 
                    'mavros_pluginlists.yaml': [os.path.join(get_package_share_directory(package_name), 'config', 'mavros_config.yaml')
                ]}.items()
    )
    
    micro_ros_agent = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            #namespace=f"{micro_ros_agent_ns}",
            output="both",
            arguments=['udp4 --port 2019'],
    )
    
    ppp = ExecuteProcess(
        cmd=[[
#            'sudo ', # sudo if not root, for 'noauth' option to work
            'pppd ', # Executable
            '/dev/ttyUSB0 ', # Serial Port
            '921600 ', # Baudrate
            '192.168.13.1:192.168.13.14 ', # local IP (Companion Computer) : remote IP (FC)
            'cdtrcts ', # Flow Control
            'dump debug ', # Debug options
            'local nodetach proxyarp noauth passive', # Additional options needed for Ardupilot PPP
            'ifname ardupilot' # Sets a nice Interface Name for the waiter...
        ]],
        shell=True
    )
    
    ppp_waiter = ExecuteProcess(
        cmd=[[
            'until (ip a show dev ardupilot); do sleep 1; done'
        ]],
        shell=True
    )
    
    mavproxy = ExecuteProcess(
        cmd=[[
            'mavproxy.py --master=udp:192.168.13.1:14550 --out=udpbcast:0.0.0.0:14550'
        ]],
        shell=True
    )

#    joystick = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource([os.path.join(
#                    get_package_share_directory(package_name),'launch','joystick.launch.py'
#                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
#    )
#
#    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
#    twist_mux = Node(
#            condition=UnlessCondition(nav),
#            package="twist_mux",
#            executable="twist_mux",
#            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
#            remappings=[('/cmd_vel_out','/cmd_vel')]
#            #remappings=[('/cmd_vel_out','/ap/cmd_vel')]
#        )
#
#    nav = GroupAction(
#        condition=IfCondition(nav),
#        actions=[
#
#            #SetRemap(src='/cmd_vel_joy',dst='/cmd_vel_teleop'),
#            #SetRemap(src='/cmd_vel',dst='/cmd_vel_out_unstamped'),
#
#            IncludeLaunchDescription(
#                    PythonLaunchDescriptionSource([os.path.join(
#                        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]), 
#                    launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_file}.items()
#            ),
#
#            Node(
#                condition=IfCondition(rviz_nav2),
#                package='rviz2',
#                executable='rviz2',
#                parameters=[{'use_sim_time': use_sim_time}],
#                arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'rviz', 'nav2_view.rviz')]
#            ),
#            
#            #IncludeLaunchDescription(
#            #        PythonLaunchDescriptionSource([os.path.join(
#            #            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]), 
#            #        launch_arguments={'use_sim_time': use_sim_time}.items(),
#            #        condition=IfCondition(slam_toolbox)
#            #),
#            
#            Node(
#                condition=IfCondition(slam_toolbox),
#                parameters=[
#                  slam_params_file,
#                  {'use_sim_time': use_sim_time}
#                ],
#                package='slam_toolbox',
#                executable='async_slam_toolbox_node',
#                name='slam_toolbox',
#                output='screen'
#            ),
#        ]
#    )
#
#    twist_stamper = Node(
#            package='twist_stamper',
#            executable='twist_stamper',
#            parameters=[{'use_sim_time': use_sim_time, 'frame_id': 'base_link'}],
#            remappings=[('/cmd_vel_in','/cmd_vel'),
#                        ('/cmd_vel_out','/ap/cmd_vel')]
#        )




    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Simulation'),
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description='Start Nav2'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'slam_toolbox',
            default_value='true',
            description='Start Slam_Toolbox\'s online_async_launch.py launch file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        DeclareLaunchArgument(
            'rviz_nav2',
            default_value='true',
            description='Start Nav2 RViz'),
        lidar,
        ppp_waiter,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ppp_waiter,
                on_exit=[
                    LogInfo(msg='PPP Interface started...'),
                    micro_ros_agent,
                    mavproxy
                ]
            )
        ),
        ppp,
        #micro_ros_agent,
        #joystick,
        #twist_mux,
        #nav,
        #twist_stamper
    ])
