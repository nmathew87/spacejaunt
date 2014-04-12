
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <igr_detection/box_viewConfig.h>
#include <algorithm>
#include <math.h>

using namespace cv;
using namespace std;


void callback(const ros::TimerEvent& event) {
  Mat image;
  image = imread("/home/prasenjit/ros_viewpoint/src/spacejaunt/images/earth_from_iss.jpg", CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    ROS_ERROR("Image not found");
  }
  else {
    imshow("Test", image);
  }
  waitKey(3); 
  ROS_INFO("Test");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_republish");
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback);
  ros::spin();
  return 0;
}
