/*

ROS Node to simulate a Ground Station

Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

<insert license statement>

*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv){
    
    ros::init(argc, argv, "gcs");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("uav/odometry", 100, gcs_odo_callback);
    ros::spin();
    
    return 0;
}

void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg){
    //stub
}