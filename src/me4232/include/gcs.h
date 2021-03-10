#ifndef _GCS_H_
#define _GCS_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <uav_obj.h>

void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg);
// geometry_msgs::TransformStamped odom_to_tf(const nav_msgs::Odometry odom);
// ros::NodeHandle n;
ros::Subscriber sub;
// tf2_ros::TransformBroadcaster br;
ros::Publisher tf_pub;
std::vector<UAVObj> UAVs; 
int count = 0;

#endif /* PID_H_ */