/*
 * YoloObjectDetector.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// c++
   #include <math.h>
   #include <string>
   #include <vector>
   #include <iostream>
   #include <pthread.h>   
   #include <thread>
   #include <chrono>
   #include <stdio.h>
        //For depth inclussion
   #include <mutex>
   #include <stdlib.h>


   #include <sstream>


   #include <cmath>
   
   // ROS
   #include <ros/ros.h>
   #include <std_msgs/Header.h>
   #include <std_msgs/Int8.h>
   #include <std_msgs/String.h>                         //For depth inclussion
   #include <actionlib/server/simple_action_server.h>   
   #include <sensor_msgs/image_encodings.h>
   #include <sensor_msgs/Image.h>

   #include <geometry_msgs/Point.h>
   #include <image_transport/image_transport.h>
   #include <message_filters/subscriber.h>                       //For depth inclussion
   #include <message_filters/synchronizer.h>                     //For depth inclussion
   #include <message_filters/time_synchronizer.h>                //For depth inclussion
   #include <message_filters/sync_policies/approximate_time.h> 
   #include <message_filters/sync_policies/exact_time.h>  //For depth inclussion
   #include <image_transport/subscriber_filter.h> 
   #include <geometry_msgs/PointStamped.h>
   #include <sensor_msgs/CameraInfo.h>
   #include <sensor_msgs/PointCloud2.h>
                  //For depth inclussion


   //Point Cloud
   #include <pcl/point_cloud.h>
   #include <pcl/point_types.h>
   #include <pcl/io/pcd_io.h>
   #include <pcl/visualization/cloud_viewer.h>
   #include <sensor_msgs/PointCloud2.h>
   #include <pcl_conversions/pcl_conversions.h>

   // OpenCv
   #include <opencv2/imgproc/imgproc.hpp>
   #include <opencv2/highgui/highgui.hpp>
   #include <opencv2/objdetect/objdetect.hpp>   
   #include <cv_bridge/cv_bridge.h>
   #include <opencv2/core/utility.hpp>          //For depth inclussion

   // darknet_ros_msgs
   #include <darknet_ros_msgs/BoundingBoxes.h>
   #include <darknet_ros_msgs/BoundingBox.h>
   #include <darknet_ros_msgs/CheckForObjectsAction.h>

// Darknet.
#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "darknet_ros/image_interface.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

extern "C" void ipl_into_image(IplImage* src, image im);
extern "C" image ipl_to_image(IplImage* src);
extern "C" void show_image_cv(image p, const char* name, IplImage* disp);
using std::vector;
namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

typedef struct {
  IplImage* image;
  std_msgs::Header header;
} IplImageWithHeader_;

class YoloObjectDetector {
 public:
  /*!
   * Constructor.
   */
  explicit YoloObjectDetector(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~YoloObjectDetector();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();
  cv::Mat lookupX, lookupY;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;

  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Callback of camera.
   * @param[in] msg image pointer.
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr&msgdepth,
  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth,const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  /*!
   * Check for objects action goal callback.
   */
  void checkForObjectsActionGoalCB();

  /*!
   * Check for objects action preempt callback.
   */
  void checkForObjectsActionPreemptCB();


  /*!
   * Check if a preempt for the check for objects action has been requested.
   * @return false if preempt has been requested or inactive.
   */
  bool isCheckingForObjects() const;

  /*!
   * Publishes the detection image.
   * @return true if successful.
   */
  bool publishDetectionImage(const cv::Mat& detectionImage);
 
  //! Using.
  using CheckForObjectsActionServer = actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction>;
  using CheckForObjectsActionServerPtr = std::shared_ptr<CheckForObjectsActionServer>;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;
  ros::AsyncSpinner spinner;
  bool updateImage, updateCloud;
  std::mutex lock;
  
  void createLookup(size_t width, size_t height);
  
  
  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

  //! Check for objects action server.
  const bool useCompressed=false;
  CheckForObjectsActionServerPtr checkForObjectsActionServer_;
  //sync subscriber
  
  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
  

  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::PCDWriter writer;
  
  const size_t queueSize;
  image_transport::ImageTransport imageTransport_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,sensor_msgs::PointCloud2> ExactSyncPolicy;
  
  image_transport::SubscriberFilter *subImageColor,*subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *subPoint;
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  //! Advertise and subscribe to image topics.
  
  void Coordinates(vector<vector<float>> position);

  //! ROS subscriber and publisher.
  image_transport::Subscriber imageSubscriber_;
  ros::Publisher objectPublisher_;
  ros::Publisher boundingBoxesPublisher_;

  //! Detected objects.
  std::vector<std::vector<RosBox_> > rosBoxes_;
  std::vector<int> rosBoxCounter_;
  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;

  //! Publisher of the bounding box image.
  ros::Publisher detectionImagePublisher_;

  // Yolo running on thread.
  std::thread yoloThread_;

  // Darknet.
  char** demoNames_;
  image** demoAlphabet_;
  int demoClasses_;

  network* net_;
  std_msgs::Header headerBuff_[3];
  image buff_[3];
  image buffLetter_[3];
  int buffId_[3];
  int buffIndex_ = 0;
  IplImage* ipl_;
  float fps_ = 0;
  float demoThresh_ = 0;
  float demoHier_ = .5;
  int running_ = 0;

  int demoDelay_ = 0;
  int demoFrame_ = 3;
  float** predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float* lastAvg2_;
  float* lastAvg_;
  float* avg_;
  int demoTotal_ = 0;
  double demoTime_;

  RosBox_* roiBoxes_;
  bool viewImage_;
  bool enableConsoleOutput_;
  int waitKeyDelay_;
  int fullScreen_;
  char* demoPrefix_;

  std_msgs::Header imageHeader_;
  cv::Mat camImageCopy_;
  cv::Mat DepthImageCopy_;
  
  boost::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  boost::shared_mutex mutexNodeStatus_;

  int actionId_;
  boost::shared_mutex mutexActionStatus_;

  // double getWallTime();

  int sizeNetwork(network* net);

  void rememberNetwork(network* net);

  detection* avgPredictions(network* net, int* nboxes);

  void* detectInThread();

  void* fetchInThread();

  void* displayInThread(void* ptr);

  void* displayLoop(void* ptr);

  void* detectLoop(void* ptr);

  void setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay, char* prefix,
                    int avg_frames, float hier, int w, int h, int frames, int fullscreen);

  void yolo();

  IplImageWithHeader_ getIplImageWithHeader();

  bool getImageStatus(void);

  bool isNodeRunning(void);

  void* publishInThread();
};

} /* namespace darknet_ros*/
