# Darknet_ROS_with_Kinect
Intergrate Darknet_ROS https://github.com/leggedrobotics/darknet_ros with Kinect V2(https://github.com/code-iai/iai_kinect2) for real distance detection with the help of Point Cloud. You should have the iai_kinect2 package and the Darknet_ROS package.

Install:

1. Replace CMakeLists.txt at  src/darknet_ros/darknet_ros
2. Replace package.xml at src/darknet_ros/darknet_ros
3. Replace YoloObjectDetector.cpp and yolo_object_detector_node.cpp at 
   src/darknet_ros/darknet_ros/src
4. Replace ros.yaml at  src/darknet_ros/darknet_ros/config

Step:

Catkin_Make
Source workspace
Terminal 1: roslaunch kinect2_bridge kinect2_bridge.launch
Terminal 2: roslaunch darknet_ros darknet_ros.launch


