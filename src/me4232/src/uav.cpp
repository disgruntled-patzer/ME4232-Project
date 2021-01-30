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

// Describe Pos and Vel for a 6 DOF model (XYZ and RPY) at a certain timestep.
// Units: m, m/s, rad, rad/s
typedef struct {
	double x,y,z,roll,pit,yaw,vx,vy,vz,v_roll,v_pit,v_yaw;
	ros::Time current_time, last_time;
} six_dof;

void compute_odom(six_dof *state);

int main(int argc, char **argv){
    
    // Anon is needed since multiple UAV nodes will be created
    ros::init(argc, argv, "uav", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("uav/odom", 100);
    ros::Rate loop_rate = 1;
    tf2_ros::TransformBroadcaster odom_broadcaster;

	six_dof state;
	state.x = 0.0;
	state.y = 0.0;
	state.z = 0.0;
	state.roll = 0.0;
	state.pit = 0.0;
	state.yaw = 0.0;
	state.vx = 0.1;
	state.vy = -0.1;
	state.vz = 0.0;
	state.v_roll = 0.0;
	state.v_pit = 0.0;
	state.v_yaw = 0.1;
	state.current_time = ros::Time::now();
	state.last_time = ros::Time::now();

    while (ros::ok()){

        // Compute odometry in a typical way given the velocities of the robot
        compute_odom(&state);

        // Since all odometry is 6DOF we'll need a quaternion created from yaw
		//geometry_msgs::Quaternion odom_quat = tf2_ros::createQuaternionMsgFromYaw(th);
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(state.v_roll,state.v_pit,state.v_yaw);

        // Publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = state.current_time;
        n.getParam("uav_id", odom_trans.header.frame_id);
        n.getParam("child_frame_id", odom_trans.child_frame_id);
        odom_trans.transform.translation.x = state.x;
		odom_trans.transform.translation.y = state.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation.x = odom_quat.x();
		odom_trans.transform.rotation.y = odom_quat.y();
		odom_trans.transform.rotation.z = odom_quat.z();
        odom_trans.transform.rotation.w = odom_quat.w();
        odom_broadcaster.sendTransform(odom_trans);

        // Publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = state.current_time;
        n.getParam("uav_id", odom.header.frame_id);

        // Set the position
		odom.pose.pose.position.x = state.x;
		odom.pose.pose.position.y = state.y;
		odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = odom_quat.x();
		odom.pose.pose.orientation.y = odom_quat.y();
		odom.pose.pose.orientation.z = odom_quat.z();
        odom.pose.pose.orientation.w = odom_quat.w();

		// Set the velocity
		n.getParam("child_frame_id", odom.child_frame_id);
		odom.twist.twist.linear.x = state.vx;
		odom.twist.twist.linear.y = state.vy;
		odom.twist.twist.angular.z = state.v_yaw;

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

// Update the 6-DOF state for each time increment
// Currently we assume 2D circular motion. @TODO: To expand to 3D motion
void compute_odom(six_dof *state){
	state->current_time = ros::Time::now();
	double dt = (state->current_time - state->last_time).toSec();
	double delta_x = (state->vx * cos(state->yaw) - state->vy * sin(state->yaw)) * dt;
	double delta_y = (state->vx * sin(state->yaw) + state->vy * cos(state->yaw)) * dt;
	double delta_yaw = state->v_yaw * dt;
	state->x += delta_x;
	state->y += delta_y;
	state->yaw += delta_yaw;
	state->last_time = state->current_time;
}