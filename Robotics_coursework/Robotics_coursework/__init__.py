#

#
"""
docker pull lcas.lincoln.ac.uk/lcas/devcontainer/ros2-teaching:2324-devel

ros2 launch uol_tidybot tidybot.launch.py


ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10


  ros2 param set /limo/gazebo_ros_depth_camera_sensor update_rate 25.0
"""