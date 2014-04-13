/*************************************************************************//**
 *****************************************************************************
 * @file        control_roi.cpp
 * @brief       Controlling ROI of image with joystick
 * @author      Neil Mathew
 * @date        2014-04-12
 *****************************************************************************
 ****************************************************************************/

#include "spacecam/main.hpp"

ros::Subscriber subscriber_img, subscriber_vp, subscriber_gamepad;

ros::Publisher publisher_roi, publisher_img;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;

Point2f vp_center;

Mat roi_final;  // roi of image being sent out
Mat full_image; // image coming up from the subscription

int localMapWidth = 640;
int localMapHeight = 480;

const char * VP_TOPIC_NAME = "/mapping/ekf/pose";

// function declarions
void genROI(Mat &image_out, Mat &image_in, Point2f &roi_center);
void imgCallback(const sensor_msgs::ImageConstPtr& msg);

double vx = 0.0;
double vy = 0.0;


// call back for new image received from cameras
void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
  
  ROS_INFO(" Full image received in imgcallback ");
  
  cv_bridge::CvImagePtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // update full_image
  full_image = cv_ptr->image;
   
}


void gamepadCallback(const geometry_msgs::Twist& cmd_vel ) {
  
    // update position
    
    
  double gamepad_x = cmd_vel.linear.y;
  double gamepad_y = cmd_vel.linear.x;

  vp_center.x = (full_image.rows / 2) * (1 + (gamepad_x/5));
  vp_center.y = (full_image.cols/ 2) * (1 + (gamepad_y/5));
  
  
  
  //imshow( "Display window", roi_final ); 
  
  // generate roi
  genROI(roi_final, full_image, vp_center);
  
  // publish message to ROS
  ros::Time time = ros::Time::now();
  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.header.frame_id = "image";
  cvi.encoding = "bgr8";
  cvi.image = roi_final;

  sensor_msgs::Image im;
  cvi.toImageMsg(im);
  image_pub_.publish(im);


}

void odoCallback( const geometry_msgs::Pose& point) {

  ROS_INFO( "Position received by odocallback : (%f, %f)", point.position.x, point.position.y);
  
  // update position
  vp_center.x = point.position.x;
  vp_center.y = point.position.y;
  
  //imshow( "Display window", roi_final ); 
  
  // generate roi
  genROI(roi_final, full_image, vp_center);
  
  // publish message to ROS
  ros::Time time = ros::Time::now();
  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.header.frame_id = "image";
  cvi.encoding = "bgr8";
  cvi.image = roi_final;

  sensor_msgs::Image im;
  cvi.toImageMsg(im);
  image_pub_.publish(im);

}

void genROI(Mat &image_out, Mat &image_in, Point2f &roi_center) // pose will be added here. 
{
    Rect roiRect;


    Point2f fov = Point2f(localMapWidth, localMapHeight);
    
	  Point2f roi;
	  roi.x = roi_center.x - fov.x/2;
	  roi.y = image_in.rows - roi_center.y - fov.y/2; 
	  
	  if (roi.x < 0 || roi.x + fov.x > image_in.cols){ cout << "incorrect region of interest "<< roi.x << endl; roi.x = 0; fov.x = 0; }
	  if (roi.y < 0 || roi.y + fov.y > image_in.rows){ cout << "incorrect region of interest "<< roi.y << endl; roi.y = 0; fov.y = 0; }
	  
	  roiRect = Rect(roi.x , roi.y, fov.x, fov.y);
	  image_out = image_in(roiRect);
    
    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image_out );   
    
}

/*
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
*/

/*****************************************************************************
 * Main
 ****************************************************************************/
int main( int argc, char** argv )
{

    // Initialize ROS
    ros::init(argc, argv, "control_roi");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it_(n);
      
    // setup publishers and subscribers  
    subscriber_vp  = n.subscribe("/viewpoint/pose", 1, odoCallback);
    subscriber_gamepad = n.subscribe("/cmd_vel",1,gamepadCallback);
    
    image_sub_ = it_.subscribe("/target_image", 1, imgCallback);
    image_pub_ = it_.advertise("/roi_out", 1);
   
    /*
    // Loading default image from command line.
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    full_image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! full_image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    */

    // Loading Default ROI
    //vp_center.x = full_image.rows / 2.0;
	  //vp_center.y = full_image.cols / 2.0;
    
    
    // genarate the first roi
    //genROI(roi_final, full_image, vp_center);
    
    
      // publish message to ROS
   // ros::Time time = ros::Time::now();
  //  cv_bridge::CvImage cvi;
  //  cvi.header.stamp = time;
  //  cvi.header.frame_id = "image";
  //  cvi.encoding = "bgr8";
  //  cvi.image = roi_final;

  //  sensor_msgs::Image im;
  //  cvi.toImageMsg(im);
  //  image_pub_.publish(im);

    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", roi_final );                   // Show our image inside it.

    ROS_INFO("Waiting for image and pose");
    
    ros::spin();
    
    //waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
