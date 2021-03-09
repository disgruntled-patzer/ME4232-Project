#ifndef _GCS_H_
#define _GCS_H_

void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg);
// geometry_msgs::TransformStamped odom_to_tf(const nav_msgs::Odometry odom);
// ros::NodeHandle n;
ros::Subscriber sub;
// tf2_ros::TransformBroadcaster br;
ros::Publisher tf_pub;

#endif /* PID_H_ */