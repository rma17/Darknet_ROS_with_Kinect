# Darknet_ROS_with_Kinect
Intergrate Darknet_ROS https://github.com/leggedrobotics/darknet_ros with Kinect V2(https://github.com/code-iai/iai_kinect2) for real distance detection with the help of Point Cloud. You should have the iai_kinect2 package and the Darknet_ROS package.

Install:

1. Replace CMakeLists.txt at  src/darknet_ros/darknet_ros
2. Replace package.xml at src/darknet_ros/darknet_ros
3. Replace YoloObjectDetector.cpp and yolo_object_detector_node.cpp at 
   src/darknet_ros/darknet_ros/src
4. Replace ros.yaml at  src/darknet_ros/darknet_ros/config

Step:

1. Catkin_Make
2. Source workspace
3. Terminal 1: roslaunch kinect2_bridge kinect2_bridge.launch
4. Terminal 2: roslaunch darknet_ros darknet_ros.launch


Note:
1. The image/pcl topic can be either /hd,/sd image or points but they should have same resolution.(/hd/points are quite slow to process on CPU).
2. Queue_size should also be identical for all the topics subscribed.

#Tested on Ubuntu 18.04 with ROS melodic


