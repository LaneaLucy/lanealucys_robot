# lanealucys_robot

### Dependency's: 
- ros2 humble
- gazebo harmonic
- [ardupilot stuff](https://ardupilot.org/dev/docs/ros2.html)
- [ardupilot gazebo stuff](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- robot_localization
- nav2_bringup
- nav2
- rviz2
- slam_toolbox
- twist_stamper
- topic_tools
- [laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher/tree/main)


### To run:
	ros2 launch lanealucys_robot robot_sim_launch.py nav2:=true slam:='slam_toolbox'
