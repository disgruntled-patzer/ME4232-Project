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

// Describe Pos and Vel for a 6 DOF model (linear XYZ and angular XYZ thetas) at a certain timestep.
// Units: m, m/s, rad, rad/s.
// The positions are relative to the GCS, or 'odom' frame, the velocities are relative to the UAV, or 'base_link' frame
// Note that xth, yth and zth is angle of the UAV wrt odom origin, not the roll/pit/yaw of UAV
typedef struct {
	double x,y,z,xth,yth,zth,vx,vy,vz,v_xth,v_yth,v_zth;
	ros::Time current_time, last_time;
} six_dof;

void init_state(six_dof *state);
void compute_odom(six_dof *state);
void update_ros_tf(six_dof *state, tf2::Quaternion *odom_quat, geometry_msgs::TransformStamped *odom_trans);
void update_ros_odom(six_dof *state, tf2::Quaternion *odom_quat, nav_msgs::Odometry *odom);

int main(int argc, char **argv){

	// Anon is needed since multiple UAV nodes will be created
	ros::init(argc, argv, "uav", ros::init_options::AnonymousName);
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("uav/odom", 100);
	ros::Rate loop_rate = 1;

	tf2_ros::TransformBroadcaster odom_broadcaster;
	tf2::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;
	nav_msgs::Odometry odom;

	six_dof state;
	init_state(&state);

	while (ros::ok()){

		// Compute odometry in a typical way given the velocities of the robot
		compute_odom(&state);

		// Since all odometry is 6DOF we'll need a quaternion created from x_theta, y_theta, z_theta
		odom_quat.setRPY(state.v_xth,state.v_yth,state.v_zth);

		// Publish the transform over tf
		n.getParam("odom_frame_id", odom_trans.header.frame_id);
		n.getParam("base_link_id", odom_trans.child_frame_id);
		update_ros_tf(&state, &odom_quat, &odom_trans);
		odom_broadcaster.sendTransform(odom_trans);

		// Publish the odometry message over ROS
		n.getParam("odom_frame_id", odom.header.frame_id);
		n.getParam("base_link_id", odom.child_frame_id);
		update_ros_odom(&state, &odom_quat, &odom);
		odom_pub.publish(odom);

		// Some logging msgs here to ensure the node is working
		ROS_INFO_STREAM(odom.child_frame_id << " to " << odom.header.frame_id);
		ROS_INFO_STREAM("Coords: [" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << "]");
		ROS_INFO_STREAM("Quat: [" << odom.pose.pose.orientation.x << ", " << odom.pose.pose.orientation.y << ", " << odom.pose.pose.orientation.z << ", " << odom.pose.pose.orientation.w << "]");
		ROS_INFO_STREAM("Linear Vel: [" << odom.twist.twist.linear.x << ", " << odom.twist.twist.linear.y << ", " << odom.twist.twist.linear.z << "]");
		ROS_INFO_STREAM("Ang Vel: [" << odom.twist.twist.angular.x << ", " << odom.twist.twist.angular.y << ", " << odom.twist.twist.angular.z << "]");

		ros::spinOnce();
		loop_rate.sleep();
	}
}

// Initialise the 6-DOF state. @TODO: Make the initialisation random/semi-random
void init_state(six_dof *state){
	state->x = state->y = state->z = 0.0;
	state->xth = state->yth = state->zth = 0.0;
	state->vx = 0.1;
	state->vy = -0.1;
	state->vz = 0.0;
	state->v_xth = 0.0;
	state->v_yth = 0.0;
	state->v_zth = 0.1;
	state->current_time = state->last_time = ros::Time::now();
}

// Update the 6-DOF odometry state for each time increment
// Currently we assume 2D circular motion. @TODO: To expand to 3D motion
void compute_odom(six_dof *state){
	state->current_time = ros::Time::now();
	double dt = (state->current_time - state->last_time).toSec();
	double delta_x = (state->vx * cos(state->zth) - state->vy * sin(state->zth)) * dt;
	double delta_y = (state->vx * sin(state->zth) + state->vy * cos(state->zth)) * dt;
	double delta_zth = state->v_zth * dt;
	state->x += delta_x;
	state->y += delta_y;
	state->zth += delta_zth;
	state->last_time = state->current_time;
}

// Update the ros transform message with data from tht 6-DOF state and quaternion
void update_ros_tf(six_dof *state, tf2::Quaternion *odom_quat, geometry_msgs::TransformStamped *odom_trans){
	odom_trans->header.stamp = state->current_time;
	odom_trans->transform.translation.x = state->x;
	odom_trans->transform.translation.y = state->y;
	odom_trans->transform.translation.z = state->z;
	odom_trans->transform.rotation.x = odom_quat->x();
	odom_trans->transform.rotation.y = odom_quat->y();
	odom_trans->transform.rotation.z = odom_quat->z();
	odom_trans->transform.rotation.w = odom_quat->w();
}

// Update the ros odometry message with data from tht 6-DOF state and quaternion
void update_ros_odom(six_dof *state, tf2::Quaternion *odom_quat, nav_msgs::Odometry *odom){

	odom->header.stamp = state->current_time;

	// Set the position
	odom->pose.pose.position.x = state->x;
	odom->pose.pose.position.y = state->y;
	odom->pose.pose.position.z = state->z;
	odom->pose.pose.orientation.x = odom_quat->x();
	odom->pose.pose.orientation.y = odom_quat->y();
	odom->pose.pose.orientation.z = odom_quat->z();
	odom->pose.pose.orientation.w = odom_quat->w();

	// Set the Velocity
	odom->twist.twist.linear.x = state->vx;
	odom->twist.twist.linear.y = state->vy;
	odom->twist.twist.linear.z = state->vz;
	odom->twist.twist.angular.x = state->v_xth;
	odom->twist.twist.angular.y = state->v_yth;
	odom->twist.twist.angular.z = state->v_zth;
}
