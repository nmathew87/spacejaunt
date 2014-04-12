
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <math.h>

using namespace cv;
using namespace std;



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_pub_ = it_.advertise("/target_image", 1);
  }

  ~ImageConverter()
  {
  }

  void imageCb(const ros::TimerEvent& event)
  {
      Mat image;
      string filename;
      nh_.param<std::string>("/image_republish/filename", filename, "image.jpg");
      image = imread(filename, CV_LOAD_IMAGE_COLOR);
      if (!image.data) {
        ROS_ERROR("Image:%s not found",filename.c_str());
      }

      ros::Time time = ros::Time::now();
      cv_bridge::CvImage cvi;
      cvi.header.stamp = time;
      cvi.header.frame_id = "image";
      cvi.encoding = "bgr8";
      cvi.image = image;

      sensor_msgs::Image im;
      cvi.toImageMsg(im);
      image_pub_.publish(im);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_republish");
  ros::NodeHandle nh;
  ImageConverter ic;
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), &ImageConverter::imageCb,&ic);
  ros::spin();
  return 0;
}
