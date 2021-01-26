/*

ROS Node to simulate a UAV in 3D space

Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

<insert license statement>

*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv){
    
    // Anon is needed since multiple UAV nodes will be created
    ros::init(argc, argv, "uav", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher odo_pub = n.advertise<nav_msgs::Odometry>("uav/odometry", 100);
    ros::Rate loop_rate = 10;

    while (ros::ok()){
        // stub. UAV must get the odometry data from somewhere, then publish to uav/odometry
        ros::spinOnce();
        loop_rate.sleep();
    }
}