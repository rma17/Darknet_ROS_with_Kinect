# Darknet_ROS_with_Kinect
Intergrate Darknet_ROS https://github.com/leggedrobotics/darknet_ros with Kinect V2(https://github.com/code-iai/iai_kinect2) for real distance detection with the help of Point Cloud. You should have the iai_kinect2 package.
1.Clone this repo to your src folder instead of the original Darknet_ROS.
2 .catkin_make.
3. Source your workspace.
4. Run roslaunch kinect2_bridge kinect2_bridge.launch .
5. In another terminal, run roslaunch darknet_ros darknet_ros.launch .
6. I have tested it on Ubuntu 18.04, ROS melodic
