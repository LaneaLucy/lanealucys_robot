import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.substitutions import Command

from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from launch.conditions import IfCondition, UnlessCondition

from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable



def generate_launch_description():

    package_name='lanealucys_robot'
    
    nav_start = LaunchConfiguration('nav2')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_toolbox = LaunchConfiguration('slam_toolbox')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('sim')
    rviz_nav2 = LaunchConfiguration('rviz_nav2')
    namespace = LaunchConfiguration('namespace')
    
    #print('namespace: ' + PythonExpression([namespace]))
    
    
    robot_description_content = ParameterValue(Command(['xacro ', PathJoinSubstitution([get_package_share_directory(package_name),'urdf','lanealucys_robot.urdf'])]), value_type=str)
    
    robot_state_publisher = Node(
            condition=UnlessCondition(use_sim_time),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
    )
    
    robot_localization_node = Node(
           package='robot_localization',
           executable='ekf_node',
           name='ekf_filter_node',
           output='screen',
           parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            condition=UnlessCondition(nav_start),
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
            #remappings=[('/cmd_vel_out','/ap/cmd_vel')]
    )
           
    repeater = Node(
        package="lanealucys_robot",
        executable="repeater",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"msgs_per_sec": 2.0},
            {"input": '/pose'},
            {"output": '/mavros/vision_pose/pose_cov'},
        ],
    )
    
    repeater_waiter = ExecuteProcess(
        cmd=[[
            'until (',
            FindExecutable(name='ros2'),
            ' node info /repeater); do sleep 1; done'
        ]],
        shell=True
    )

    nav = GroupAction(
        condition=IfCondition(nav_start),
        actions=[
            PushRosNamespace(namespace),

            #SetRemap(src='/cmd_vel_joy',dst='/cmd_vel_teleop'),
            #SetRemap(src='/cmd_vel',dst='/cmd_vel_out_unstamped'),

            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]), 
                    launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items() #, 'params_file': nav2_params_file}.items()
            ),

            Node(
                condition=IfCondition(rviz_nav2),
                package='rviz2',
                executable='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'rviz', 'nav2_view.rviz')]
            ),
            
            #IncludeLaunchDescription(
            #        PythonLaunchDescriptionSource([os.path.join(
            #            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]), 
            #        launch_arguments={'use_sim_time': use_sim_time}.items(),
            #        condition=IfCondition(slam_toolbox)
            #),
            
            Node(
                condition=IfCondition(slam_toolbox),
                parameters=[
                  slam_params_file,
                  {'use_sim_time': use_sim_time}
                ],
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen'
            ),
            
            #Node(
            #    condition=UnlessCondition(slam_toolbox),
            #    package="cartographer_ros",
            #    executable="cartographer_node",
            #    parameters=[{"use_sim_time": use_sim_time}],
            #    arguments=[
            #        "-configuration_directory",
            #        os.path.join(get_package_share_directory(package_name), 'config'),
            #        "-configuration_basename",
            #        "cartographer.lua",
            #    ],
            #    output="screen",
            #    remappings=[
            #        ("/imu", "/mavros/imu/data"),
            #        ("/odom", "/mavros/local_position/odom"),
            #    ],
            #),
            #Node(
            #    condition=UnlessCondition(slam_toolbox),
            #    package="cartographer_ros",
            #    executable="cartographer_occupancy_grid_node",
            #    parameters=[
            #        {"use_sim_time": use_sim_time},
            #        {"resolution": 0.05},
            #    ],
            #),
        ]
    )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': use_sim_time, 'frame_id': 'base_link'}],
            remappings=[('/cmd_vel_in','/cmd_vel'),
                        ('/cmd_vel_out','/mavros/setpoint_velocity/cmd_vel')]
    )
    
    robot = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            robot_state_publisher,
            robot_localization_node,
            repeater,
            repeater_waiter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=repeater_waiter,
                    on_exit=[
                        LogInfo(msg='repeater started...'),
                        nav
                    ]
                )
            ),
            #nav
            #joystick,
            #twist_mux,
            twist_stamper
        ]
    )




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
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='namespace'),
        LogInfo(msg=namespace),
        robot
    ])
