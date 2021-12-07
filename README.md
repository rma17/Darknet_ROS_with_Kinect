# Darknet_ROS_with_Kinect
Intergrate Darknet_ROS https://github.com/leggedrobotics/darknet_ros with Kinect V2(https://github.com/code-iai/iai_kinect2) for real distance detection with the help of Point Cloud.
clone this repo to your src folder
catkin_make
source your workspace
Run roslaunch kinect2_bridge kinect2_bridge.launch
In another terminal, run roslaunch darknet_ros darknet_ros.launch
