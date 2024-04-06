#

#
"""
Detect wall:
https://get-help.theconstruct.ai/t/2-1-create-service-server-how-to-update-scan-values-in-while-loop-in-find-wall-service/23382


docker pull lcas.lincoln.ac.uk/lcas/devcontainer/ros2-teaching:2324-devel

ros2 launch uol_tidybot tidybot.launch.py


ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 2


  ros2 param set /limo/gazebo_ros_depth_camera_sensor update_rate 25.0
"""
