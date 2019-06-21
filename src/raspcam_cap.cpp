/**
 * ------------------------------------------------ *
 * camera_capture.cpp
 * MIT License
 * Copyright (c) 2019 Yuntian Li
 * ------------------------------------------------ *
 * A ros node for capturing image with embedded MPI
 * camera and publish in color format.
 * ------------------------------------------------ *
 * Tested with Jetson Nano and Raspberry Cam V2.1 8MP
 */
// headfiles of std c++
#include <iostream>
// headfiles of ros libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// headfiles of opencv libraries
#include <opencv2/opencv.hpp>

//declaration of pipeline generation function
std::string gstream_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_mode);
 
int main(int argc, char** argv){
  // initialize ros node
  ros::init(argc, argv, "RaspCam_publisher");
  // create rosnode handle
  ros::NodeHandle nh;
  // creste image_transport 
  image_transport::ImageTransport it(nh);
  // create image publisher
  image_transport::Publisher img_pub = it.advertise("camera/RaspCam", 1, true);

  // set arguments for gstream pipeline
  /*
	int capture_width = 1280;
	int capture_height = 720;
	int display_width = 1280;
	int display_height = 720;
	int framerate = 30;
	int flip_mode = 2;
  */

  // set parameters with roslaunch parameter server
  int capture_width, capture_height;
  int display_width, display_height;
  int framerate;
  int flip_mode;
  // use handle to get parameters from paramter server
  nh.getParam("/raspcam_capture/capture_width", capture_width);
  ROS_INFO("Get param %d", capture_width);
  nh.getParam("/raspcam_capture/capture_height", capture_height);
  nh.getParam("/raspcam_capture/display_width", capture_width);
  nh.getParam("/raspcam_capture/display_height", capture_height);
  nh.getParam("/raspcam_capture/framerate", framerate);
  nh.getParam("/raspcam_capture/flip_mode", flip_mode);
  
  std::string pipeline = gstream_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_mode);



  std::cout << "Using pipeline: " << pipeline << "for gstream\n";
  // define capture instance and check status
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
	if(!cap.isOpened()) {
	  ROS_ERROR("Faild to open gstream with current pipeline, please check carefully !");
	  return (-1);
	}
  // ros publish rate;
  ros::Rate rate(framerate);
  // cv::Mat instance to store capture frames;
  cv::Mat frame;
  // imageptr for ros image message
  sensor_msgs::ImagePtr msg;

  while(ros::ok()){
    cap  >> frame;
    if (!frame.empty()){
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      img_pub.publish(msg);
      cv::waitKey(1);
    }
    ros::spinOnce();
    rate.sleep();
  }



}

std::string gstream_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_mode){
  // function for gstream pipeline defination
  
	std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)"
		+ std::to_string(capture_width)
		+ ", height=(int)"
		+ std::to_string(capture_height)
		+ ", format=(string)NV12, framerate=(fraction)"
		+ std::to_string(framerate)
		+ "/1 ! nvvidconv flip-method="
		+ std::to_string(flip_mode) 
		+ " ! video/x-raw, width=(int)"
		+ std::to_string(display_width)
		+ ", height=(int)"
		+ std::to_string(display_height)
		+ ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  // set camera capture image and store in NVMM memory;
  // pipeline :sensor -> NVMM
	return pipeline;
}


