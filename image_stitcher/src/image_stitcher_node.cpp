#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/stitcher.hpp"    

using namespace std;
namespace enc = sensor_msgs::image_encodings;
typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;


class ImageStitcher
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ImageSubscriber* left_image_sub;
  ImageSubscriber* right_image_sub;
  message_filters::Synchronizer<syncPolicy>* sync;
  cv::Mat pano;
  std::vector<cv::Mat> imgs;
  bool registered;

public:
  void imageCallback(const sensor_msgs::ImageConstPtr& left_ptr, 
    const sensor_msgs::ImageConstPtr& right_ptr)
  {
    cv_bridge::CvImagePtr left_img_cvptr, right_img_cvptr;
    try {
      left_img_cvptr = cv_bridge::toCvCopy(left_ptr, sensor_msgs::image_encodings::BGR8);
      right_img_cvptr = cv_bridge::toCvCopy(right_ptr, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    imgs[0] = left_img_cvptr->image;
    imgs[1] = right_img_cvptr->image;
   
    static cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
    
    if(!registered)
    {
      cv::Stitcher::Status status = stitcher.estimateTransform(imgs); 
      if(status == cv::Stitcher::OK) {
        registered = true;
      }
    }

    if(registered) {
      cv::Stitcher::Status status = stitcher.composePanorama(imgs, pano);
    }
    cv::imshow("stitched image", pano);
    cv::imshow("left image", imgs[0]);
    cv::imshow("right image", imgs[1]);
    cv::waitKey(3);
  }


  ImageStitcher():it_(nh_)
  { 
    left_image_sub = new ImageSubscriber(it_, "left/image_raw", 1);
    right_image_sub = new ImageSubscriber(it_, "right/image_raw", 1);
    sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *left_image_sub, *right_image_sub);
    sync->registerCallback(boost::bind(&ImageStitcher::imageCallback, this, _1, _2));
    imgs.resize(2);
    cv::namedWindow("stitched image", cv::WINDOW_AUTOSIZE );
    cv::namedWindow("left image", cv::WINDOW_AUTOSIZE );
    cv::namedWindow("right image", cv::WINDOW_AUTOSIZE );
    registered = false;
  }

  ~ImageStitcher()
  {
    delete left_image_sub;
    delete right_image_sub;
    delete sync;
  }
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_stitcher");

  // Convert Image
  ImageStitcher ic;
  ros::spin();
  return 0;
}



