#include <spacecam/main.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::Subscriber subscriber_qt;
ros::Publisher publisher_pose;
ros::Publisher pose_pub;
double full_image_width = 4256;
double full_image_height = 2832;

void qt_Callback( const geometry_msgs::Quaternion& oculus_qt) {
  double roll, pitch, yaw;

  tf::Quaternion q;
  tf::quaternionMsgToTF(oculus_qt, q);
  tf::Quaternion q_t (-q.x(), -q.z(), -q.w(), q.y());

  tf::Matrix3x3(q_t).getRPY(roll, pitch, yaw);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation(q_t);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "oculus"));
  
  double x = (-yaw/3.14159)*(full_image_width/2.0) + (full_image_width/2.0);
  double y = (pitch/3.14159)*(full_image_height/2.0) + (full_image_height/2.0);
  double z = 0;

  double oc_pitch = roll;
  geometry_msgs::Pose output_pose;
  output_pose.position.x = x;
  output_pose.position.y = y;
  output_pose.position.z = z;
  output_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,oc_pitch,0);
  pose_pub.publish(output_pose);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "publish_roi_center");
    ros::NodeHandle n;
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber sub = n.subscribe("oculus/orientation", 1000, qt_Callback);  
    pose_pub = n.advertise<geometry_msgs::Pose>("viewpoint/pose",1000);
    ros::spin();
}
