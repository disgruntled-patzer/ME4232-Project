/*

   ROS Node to convert UAV odom pos to Rviz

   Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

   <insert license statement>

*/

#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class odom_to_rviz{
    private:
        visualization_msgs::Marker marker;
    
    public:

        ros::NodeHandle n;
        ros::Publisher pub_to_rviz = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        odom_to_rviz();
        void rviz_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void rviz_handler();
};

// Init the permanent rviz marker properties
odom_to_rviz::odom_to_rviz(){
    
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "uav";
    n.getParam("odom_frame_id", marker.header.frame_id);
    
    // Marker size
    marker.scale.x = 0.4;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Marker colour
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}

// Callback to receive odom data from UAV and forward the XYZ coords to rviz
void odom_to_rviz::rviz_callback(const nav_msgs::Odometry::ConstPtr& msg){
    
    // Get the UAV id so that we can update the correct marker
    std::stringstream ss(msg->child_frame_id); // Convert str to int using sstream
    ss >> marker.id;

    // XYZ pos
    marker.pose.position.x = msg->pose.pose.position.x;
    marker.pose.position.y = msg->pose.pose.position.y;
    marker.pose.position.z = msg->pose.pose.position.z;

    // Orientation
    marker.pose.orientation.x = msg->pose.pose.orientation.x;
    marker.pose.orientation.y = msg->pose.pose.orientation.y;
    marker.pose.orientation.z = msg->pose.pose.orientation.z;
    marker.pose.orientation.w = msg->pose.pose.orientation.w;
    
    marker.header.stamp = ros::Time::now();

    pub_to_rviz.publish(marker);
}

// "Main" function to run the me4232_rviz node
void odom_to_rviz::rviz_handler(){
    ros::Subscriber sub = n.subscribe("uav/odom", 100, &odom_to_rviz::rviz_callback, this);
    ros::spin();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "me4232_rviz");
    odom_to_rviz odom_to_rviz_instance;
    odom_to_rviz_instance.rviz_handler();

}