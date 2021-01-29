/*

ROS Node to simulate a UAV in 3D space.

Basic tf code is adopted and modified from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

<insert license statement>

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv){
    
    // Anon is needed since multiple UAV nodes will be created
    ros::init(argc, argv, "uav", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("uav/odom", 100);
    ros::Rate loop_rate = 1;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    // xy + angular (th) pos and vel. Units: m/s and rad/s
    double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;

    ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    while (ros::ok()){

        // Compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		x += delta_x;
		y += delta_y;
		th += delta_th;

        // Since all odometry is 6DOF we'll need a quaternion created from yaw
		//geometry_msgs::Quaternion odom_quat = tf2_ros::createQuaternionMsgFromYaw(th);
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0,0,th);

        // Publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
        n.getParam("uav_id", odom_trans.header.frame_id);
        n.getParam("child_frame_id", odom_trans.child_frame_id);
        odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation.x = odom_quat.x();
		odom_trans.transform.rotation.y = odom_quat.y();
		odom_trans.transform.rotation.z = odom_quat.z();
        odom_trans.transform.rotation.w = odom_quat.w();
        odom_broadcaster.sendTransform(odom_trans);

        // Publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
        n.getParam("uav_id", odom.header.frame_id);

        // Set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = odom_quat.x();
		odom.pose.pose.orientation.y = odom_quat.y();
		odom.pose.pose.orientation.z = odom_quat.z();
        odom.pose.pose.orientation.w = odom_quat.w();

		// Set the velocity
		n.getParam("child_frame_id", odom.child_frame_id);
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		// Publish the message
		odom_pub.publish(odom);

        // Some logging msgs here to ensure the node is working
        ROS_INFO_STREAM(odom.header.frame_id << " to " << odom.child_frame_id);
        ROS_INFO_STREAM("Coords: [" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << "]");
        ROS_INFO_STREAM("Vel: [" << odom.twist.twist.linear.x << ", " << odom.twist.twist.linear.y << ", " << odom.twist.twist.angular.z << "]");

        ros::spinOnce();
        loop_rate.sleep();
    }
}