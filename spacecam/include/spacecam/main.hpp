
/*************************************************************************//**
 *****************************************************************************
 * @file        main.hpp
 * @brief       
 * @author      Frank Imeson
 * @date        2012-01-02
 *****************************************************************************
 ****************************************************************************/

// http://www.ros.org/wiki/CppStyleGuide
// http://www.parashift.com/c++-faq-lite/
// http://www.emacswiki.org/emacs/KeyboardMacrosTricks



/*****************************************************************************
 * INCLUDE
 ****************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <sstream>
#include <vector>	
#include <queue>
#include <map>
#include <iostream>
#include <Eigen/Core>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>

//#include <boost/geometry.hpp> 
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>

// Need to change this to correct package inclusion method


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <actionlib/server/simple_action_server.h>

//#include "general.hpp"
//#include "map_tools.hpp"
//#include "greedycut.hpp"
//#include "RobotStates.h"

using namespace cv;
using namespace std;
using namespace Eigen;


/*****************************************************************************
 * DEFINE
 ****************************************************************************/





