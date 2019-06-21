#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    cv::Mat img_receive = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (!img_receive.data){
      ROS_INFO("Null image, waiting for next frame.");
    }
    else{
      cv::imshow("ORB", img_receive);
      /*
      cv::Mat img_toshow_orb = img_receive.clone();
      cv::Mat img_toshow_fast = img_receive.clone();


      cv::Ptr<cv::ORB> orb = cv::ORB::create();
      cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create();
      
      std::vector<cv::KeyPoint> keypoints1, keypoints2;
      
      orb->setEdgeThreshold(10);

      orb->detect(img_receive, keypoints1);
      
      fast->detect(img_receive, keypoints2);
      
      cv::drawKeypoints(img_receive, keypoints1, img_toshow_orb, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      cv::drawKeypoints(img_receive, keypoints2, img_toshow_fast, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

      cv::imshow("ORB", img_toshow_orb);
      cv::imshow("FAST", img_toshow_fast);
    */
    }
    // Some ros tutorial may ask you to add a waitkey here
    // but it may cause 'G_IS_OBJECT' error
    // If that happened to you, comment this line.
    //cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "img_subscriber");
  ros::NodeHandle nh;
  
  // 创建OpenCV显示窗口
  cv::namedWindow("ORB", CV_WINDOW_AUTOSIZE);
  // 此语句未发现有何作用，官方文档也未说明
  cv::startWindowThread();
  
  cv::namedWindow("FAST", CV_WINDOW_AUTOSIZE);
  cv::startWindowThread();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/camera/RaspCam", 1, imageCallback);
  ros::spin();
  //节点停止时摧毁显示窗口
  cv::destroyWindow("ORB");
  cv::destroyWindow("FAST");

  return 0;
}
