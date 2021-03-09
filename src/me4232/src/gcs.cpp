/*

ROS Node to simulate a Ground Station

Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

<insert license statement>

*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gcs.h>
#include <uav_obj.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "gcs");

    ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe("uav/odom", 100, gcs_odo_callback);
//     ros::spin();
    sub = n.subscribe("uav/odometry", 100, gcs_odo_callback);
    // tf_pub = n.advertise<geometry_msgs::TransformStamped>("uav/tf_stamped", 100);
    UAVObj a(1., 1.);
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 1;
    msg.pose.pose.position.z = 2;
    msg.pose.pose.orientation.w = 1;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    a.set_odom(msg);
    UAVObj b(1., 1.);
    nav_msgs::Odometry msg_2;
    msg_2.pose.pose.position.x = 10;
    msg_2.pose.pose.position.y = 1;
    msg_2.pose.pose.position.z = 2;
    msg_2.pose.pose.orientation.w = 1;
    msg_2.pose.pose.orientation.x = 0;
    msg_2.pose.pose.orientation.y = 0;
    msg_2.pose.pose.orientation.z = 0;
    b.set_odom(msg_2);
    bool result = a.check_interference(b);
    std::cout << "Will hit: " << result << std::endl;
    // std::cout << a.eigen << std::endl;
    // std::cout << b.eigen << std::endl;
    // ros::spin();
    
    return 0;
}


void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg){
    //stub
    // tf_pub.publish(odom_to_tf(*msg));
}


// geometry_msgs::TransformStamped odom_to_tf(const nav_msgs::Odometry odom){

//     geometry_msgs::TransformStamped transform_stamped;
//     transform_stamped.header.stamp = odom.header.stamp;
//     // transform_stamped.header.frame_id = frame_id;
//     // transform_stamped.child_frame_id = child_frame_id;
//     transform_stamped.transform.translation.x = odom.pose.pose.position.x;
//     transform_stamped.transform.translation.y = odom.pose.pose.position.y;
//     transform_stamped.transform.translation.z = odom.pose.pose.position.z;
//     transform_stamped.transform.rotation = odom.pose.pose.orientation;
//     return transform_stamped;
// }
