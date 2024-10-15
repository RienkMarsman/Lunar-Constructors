# Lunar-Constructors
to launch the model
 - chmod +x launch_rover.sh (once)
 - ./launch_rover

requirements:
- ROS2 
- GAZEBO 
- tmux


run this to see raw_camera output
 ros2 run rqt_image_view rqt_image_view /camera1/image_raw_view