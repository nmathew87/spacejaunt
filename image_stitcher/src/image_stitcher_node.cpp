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
  ImageSubscriber* up_image_sub;
  ImageSubscriber* down_image_sub;
  message_filters::Synchronizer<syncPolicy>* sync;
  cv::Mat pano;
  std::vector<cv::Mat> imgs;
  
  image_transport::Publisher image_pub_;

public:
  void imageCallback(const sensor_msgs::ImageConstPtr& up_ptr, 
    const sensor_msgs::ImageConstPtr& down_ptr)
  {
    cv_bridge::CvImagePtr up_img_cvptr, down_img_cvptr;
    try {
      up_img_cvptr = cv_bridge::toCvCopy(up_ptr, sensor_msgs::image_encodings::BGR8);
      down_img_cvptr = cv_bridge::toCvCopy(down_ptr, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    imgs[0] = up_img_cvptr->image;
    imgs[1] = down_img_cvptr->image;
   
    cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
    cv::Stitcher::Status status = stitcher.stitch(imgs, pano); 
    
    // publishing pano image
    ros::Time time = ros::Time::now();
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = pano;  // storing pano in cv image

    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub_.publish(im);
  }


  ImageStitcher():it_(nh_)
  { 
  
    image_pub_ = it_.advertise("/stitched_image", 1);
  
    up_image_sub = new ImageSubscriber(it_, "left/image_raw", 1);
    down_image_sub = new ImageSubscriber(it_, "right/image_raw", 1);
    sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *up_image_sub, *down_image_sub);
    sync->registerCallback(boost::bind(&ImageStitcher::imageCallback, this, _1, _2));
    imgs.resize(2);
  }

  ~ImageStitcher()
  {
    delete up_image_sub;
    delete down_image_sub;
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


