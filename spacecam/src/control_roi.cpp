/*************************************************************************//**
 *****************************************************************************
 * @file        control_roi.cpp
 * @brief       Controlling ROI of image with joystick
 * @author      Neil Mathew
 * @date        2014-04-12
 *****************************************************************************
 ****************************************************************************/

#include "spacecam/main.hpp"

ros::Subscriber subscriber_img, subscriber_vp;
ros::Publisher publisher_roi, publisher_img;

Point2f vp_center;

const char * VP_TOPIC_NAME = "/mapping/ekf/pose";



void odoCallback( const geometry_msgs::TwistConstPtr& cmd_vel) {

  ROS_INFO( "Velocity_Recieved_by_OdomNode");
  vx = cmd_vel->linear.x;
  vy = cmd_vel->linear.y;
  vth = cmd_vel->angular.z; // don't need angular

  // update position
  vp_center.x = vp_center.x + vx;
  vp_center.y = vp_center.y + vy;
  
  cout<< "vp_center moved to" << vp_center.x <<endl;
}

void showROI(Mat &full_image) {

    Mat roi_out;
    Rect roiRect;

	  int localMapWidth = 500;
	  int localMapHeight = 300;
    Point2f fov = Point2f(localMapWidth, localMapHeight);

	  vp_center.x = 2500;
	  vp_center.y = 1500;
	  
	  Point2f roi;
	  roi.x = vp_center.x - fov.x/2;
	  roi.y = full_image.rows - vp_center.y - fov.y/2; 
	  
	  if (roi.x < 0 || roi.x + fov.x > full_image.cols){ cout << "incorrect region of interest "<< roi.x << endl; roi.x = 0; fov.x = 0; }
	  if (roi.y < 0 || roi.y + fov.y > full_image.rows){ cout << "incorrect region of interest "<< roi.y << endl; roi.y = 0; fov.y = 0; }
	  roiRect = Rect(roi.x , roi.y, fov.x, fov.y);
	  roi_out = full_image(roiRect);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", roi_out );   

}




 
/*****************************************************************************
 * Main
 ****************************************************************************/
int main( int argc, char** argv )
{
    ros::init(argc, argv, "control_roi");

    ros::NodeHandle n;
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, odoCallback);


    // Loading image from command line. Change to ros subscriber!!
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat full_image;
    full_image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! full_image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    
    // image is obtained. Start with ROI stuff.    
    
    Mat roi_out;
    Rect roiRect;

	  int localMapWidth = 500;
	  int localMapHeight = 300;
    Point2f fov = Point2f(localMapWidth, localMapHeight);

	  vp_center.x = 2500;
	  vp_center.y = 1500;
	  
	  Point2f roi;
	  roi.x = vp_center.x - fov.x/2;
	  roi.y = full_image.rows - vp_center.y - fov.y/2; 
	  
	  if (roi.x < 0 || roi.x + fov.x > full_image.cols){ cout << "incorrect region of interest "<< roi.x << endl; roi.x = 0; fov.x = 0; }
	  if (roi.y < 0 || roi.y + fov.y > full_image.rows){ cout << "incorrect region of interest "<< roi.y << endl; roi.y = 0; fov.y = 0; }
	  roiRect = Rect(roi.x , roi.y, fov.x, fov.y);
	  roi_out = full_image(roiRect);

    


    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", roi_out );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
