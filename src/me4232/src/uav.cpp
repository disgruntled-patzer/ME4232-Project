/*

   ROS Node to simulate a UAV in 3D space.

   Basic tf code is adopted and modified from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

   Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

   <insert license statement>

*/

#include <fstream>
#include <math.h>
#include <sstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define MAX_TRAJ 7 // Max number of CSV trajectory files
#define MAX_INITIAL_POS 5 // Max possible value of starting XYZ coordinates
#define MAX_ACC 1 // Max possible IMU acceleration
#define MAX_ANG_VEL 0.5 // Max possible gyro angular velocity
// #define CSV // Get IMU/gyro data from a selected CSV file. If this macro is off, the IMU/gyro data is generated randomly
// #define DEBUG

// Describe Pos and Vel for a 6 DOF model (XYZ and RPY) at a certain timestep.
// Units: m, m/s, rad, rad/s.
// The linear positions are relative to the GCS, or 'odom' frame, all others relative to the UAV, or 'base_link' frame
typedef struct {
	double x,y,z,xth,yth,zth,vx,vy,vz,v_xth,v_yth,v_zth;
	ros::Time current_time, last_time;
} six_dof;

// Data that is returned from a typical IMU/Gyroscope sensor (linear accel and rotational speeds in 3D)
typedef struct {
	double ax, ay, az, v_xth, v_yth, v_zth;
} imu_data;

class uav_odom{
	
	private:

		tf2_ros::TransformBroadcaster odom_broadcaster;
		tf2::Quaternion odom_quat;
		geometry_msgs::TransformStamped odom_trans;
		nav_msgs::Odometry odom;
		six_dof state;
		imu_data reading;

		// CSV trajectory file handlers
		std::string imu_data_src; // Location of csv file containing imu data
		std::fstream reader; // File reader
		std::string line; // To read each line from the csv file
	
	public:

		ros::NodeHandle n;
		ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("uav/odom", 100);
		ros::Rate loop_rate = 10;

		uav_odom();
		void access_imu_data();
		void extract_imu_data();
		void compute_odom();
		void quat_rotate(double &x, double &y, double &z, double xth, double yth, double zth);
		void update_ros_tf();
		void update_ros_odom();
		void log_odom();
		void odom_manager();
};

// Initialise the 6-DOF state
uav_odom::uav_odom(){
	// Starting XYZ position is random
	srand(time(0));
	state.x = double(rand() % MAX_INITIAL_POS);
	state.y = double(rand() % MAX_INITIAL_POS);
	state.z = double(rand() % MAX_INITIAL_POS);
	// All other starting values are zero
	state.xth = state.yth = state.zth = 0.0;
	state.vx = state.vy = state.vz = 0.0;
	state.v_xth = state.v_yth = state.v_zth = 0.0;
	state.current_time = state.last_time = ros::Time::now();
}

// Randomly select and open a CSV trajectory file
void uav_odom::access_imu_data(){
	std::string uav_id, location;
	n.getParam("base_link_id", uav_id);
	n.getParam("imu_data_location", location); // Location to csv files
	srand(time(0));
	int filename;
	std::stringstream ss;
	#ifdef DEBUG
		n.getParam("filename", filename);
	#else
		filename = rand() % MAX_TRAJ; // Filename is a random number from 0 to (MAX_TRAJ - 1)
	#endif
	ss << location << filename << ".csv"; // Assemble the full directory to the file
	imu_data_src = ss.str();
	ROS_INFO_STREAM("UAV " << uav_id << " opening file " << imu_data_src);
	// Once filename is generated, open it
	reader.open(imu_data_src, std::fstream::in);
	if (!reader.is_open()){
		throw std::runtime_error("UAV: Could not read IMU data");
	}
	std::getline(reader, line); // Read and discard first row (which contains headers)
}

// Extract IMU data and put into the 6-DOF state
void uav_odom::extract_imu_data(){
	#ifdef CSV // Extract data from CSV file
		std::stringstream ss(line);
		ss >> reading.ax;
		ss.ignore();
		ss >> reading.ay;
		ss.ignore();
		ss >> reading.az;
		ss.ignore();
		ss >> reading.v_xth;
		ss.ignore();
		ss >> reading.v_yth;
		ss.ignore();
		ss >> reading.v_zth;
		ss.ignore();
	#else // Generate random data between 0 - MAX_ACC (for accel) and 0 - MAX_ANG_VEL (for angular velocity)
		reading.ax = (double)rand()/RAND_MAX*MAX_ACC;
		reading.ay = (double)rand()/RAND_MAX*MAX_ACC;
		reading.az = (double)rand()/RAND_MAX*MAX_ACC;
		reading.v_xth = (double)rand()/RAND_MAX*MAX_ANG_VEL;
		reading.v_yth = (double)rand()/RAND_MAX*MAX_ANG_VEL;
		reading.v_zth = (double)rand()/RAND_MAX*MAX_ANG_VEL;
	#endif
}

// Update the 6-DOF odometry state for each time increment using the IMU linear accel + gyro angular vel reaidngs
void uav_odom::compute_odom(){
	state.current_time = ros::Time::now();
	double dt = (state.current_time - state.last_time).toSec();
	// RPY Rates
	state.v_xth = reading.v_xth;
	state.v_yth = reading.v_yth;
	state.v_zth = reading.v_zth;
	// XYZ Rates
	double dvx = reading.ax*dt;
	double dvy = reading.ay*dt;
	double dvz = reading.az*dt;
	state.vx += dvx;
	state.vy += dvy;
	state.vz += dvz;
	// RPY
	double dxth = reading.v_xth*dt;
	double dyth = reading.v_yth*dt;
	double dzth = reading.v_zth*dt;
	state.xth += dxth;
	state.yth += dyth;
	state.zth += dzth;
	// To calculate XYZ, first calculate linear motion, then add angular motion
	double dx = state.vx*dt; // Add linear motion
	double dy = state.vy*dt;
	double dz = state.vz*dt;
	if (state.xth || state.yth || state.zth){
		quat_rotate(dx, dy, dz, state.xth, state.yth, state.zth); // Transform linear increments from uav to gcs frame
	}
	state.x += dx;
	state.y += dy;
	state.z += dz;
	if (dxth || dyth || dzth){
		quat_rotate(state.x, state.y, state.z, dxth, dyth, dzth); // Add angular motion if needed
	}
	state.last_time = state.current_time;
}

// Takes in references to XYZ coords and values of RPY, and perform a 3-axis quaternion rotation on the XYZ coords
// The convention used is Z-Y-X rotation, and the rotation angles are Tait-Bryan "Euler" angles (aka RPY)
void uav_odom::quat_rotate(double &x, double &y, double &z, double xth, double yth, double zth){
	tf2::Quaternion quat_rot, quat_rot_conj, quat_result;
	quat_result.setValue(x, y, z);
	#ifdef DEBUG
		ROS_INFO_STREAM("quad_old: " << quat_result.w() << ", " << quat_result.x() << ", " << quat_result.y() << ", " << quat_result.z());
	#endif
	if (zth){ // First rotate around z...
		quat_rot.setValue(0, 0, sin(zth/2), cos(zth/2));
		quat_rot_conj.setValue(-quat_rot.x(), -quat_rot.y(), -quat_rot.z(), quat_rot.w());
		quat_result = quat_rot * quat_result * quat_rot_conj;
	}
	if (yth){ /// Then y...
		quat_rot.setValue(0, sin(yth/2), 0, cos(yth/2));
		quat_rot_conj.setValue(-quat_rot.x(), -quat_rot.y(), -quat_rot.z(), quat_rot.w());
		quat_result = quat_rot * quat_result * quat_rot_conj;
	}
	if (xth){ // And finally x
		quat_rot.setValue(sin(xth/2), 0, 0, cos(xth/2));
		quat_rot_conj.setValue(-quat_rot.x(), -quat_rot.y(), -quat_rot.z(), quat_rot.w());
		quat_result = quat_rot * quat_result * quat_rot_conj;
	}
	#ifdef DEBUG
		ROS_INFO_STREAM("quad_new: " << quat_result.w() << ", " << quat_result.x() << ", " << quat_result.y() << ", " << quat_result.z());
	#endif
	x = quat_result.x(); //"Return" the transformed XYZ coords through pass-by-reference
	y = quat_result.y();
	z = quat_result.z();
}

// Update the ros transform message with data from tht 6-DOF state and quaternion
void uav_odom::update_ros_tf(){
	n.getParam("odom_frame_id", odom_trans.header.frame_id);
	n.getParam("base_link_id", odom_trans.child_frame_id);
	odom_trans.header.stamp = state.current_time;
	odom_trans.transform.translation.x = state.x;
	odom_trans.transform.translation.y = state.y;
	odom_trans.transform.translation.z = state.z;
	odom_trans.transform.rotation.x = odom_quat.x();
	odom_trans.transform.rotation.y = odom_quat.y();
	odom_trans.transform.rotation.z = odom_quat.z();
	odom_trans.transform.rotation.w = odom_quat.w();
}

// Update the ros odometry message with data from tht 6-DOF state and quaternion
void uav_odom::update_ros_odom(){

	n.getParam("odom_frame_id", odom.header.frame_id);
	n.getParam("base_link_id", odom.child_frame_id);
	odom.header.stamp = state.current_time;

	// Set the position
	odom.pose.pose.position.x = state.x;
	odom.pose.pose.position.y = state.y;
	odom.pose.pose.position.z = state.z;
	odom.pose.pose.orientation.x = odom_quat.x();
	odom.pose.pose.orientation.y = odom_quat.y();
	odom.pose.pose.orientation.z = odom_quat.z();
	odom.pose.pose.orientation.w = odom_quat.w();

	// Set the velocity
	odom.twist.twist.linear.x = state.vx;
	odom.twist.twist.linear.y = state.vy;
	odom.twist.twist.linear.z = state.vz;
	odom.twist.twist.angular.x = state.v_xth;
	odom.twist.twist.angular.y = state.v_yth;
	odom.twist.twist.angular.z = state.v_zth;
}

// Log the odom data in roslog
void uav_odom::log_odom(){
	ROS_INFO_STREAM(odom.child_frame_id << " to " << odom.header.frame_id);
	ROS_INFO_STREAM("Coords: [" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << "]");
	ROS_INFO_STREAM("Quat: [" << odom.pose.pose.orientation.w << ", " << odom.pose.pose.orientation.y << ", " << odom.pose.pose.orientation.y << ", " << odom.pose.pose.orientation.z << "]");
	ROS_INFO_STREAM("Linear Vel: [" << odom.twist.twist.linear.x << ", " << odom.twist.twist.linear.y << ", " << odom.twist.twist.linear.z << "]");
	ROS_INFO_STREAM("Ang Vel: [" << odom.twist.twist.angular.x << ", " << odom.twist.twist.angular.y << ", " << odom.twist.twist.angular.z << "]");
}

// "Main" function to broadcast tf and odom msgs to GCS and receive cmds
void uav_odom::odom_manager(){

	#ifdef CSV
		access_imu_data();
	#endif

	while (ros::ok()){

		// Break if end of csv file reached
		#ifdef CSV
			if (!std::getline(reader, line)){
				break;
			}
		#endif

		// Extract imu data from the specified csv file and compute odometry
		extract_imu_data();
		compute_odom();

		// Since all odometry is 6DOF we'll need a quaternion created from x_theta, y_theta, z_theta
		odom_quat.setRPY(state.xth,state.yth,state.zth);

		// Publish the transform over tf
		update_ros_tf();
		odom_broadcaster.sendTransform(odom_trans);

		// Publish the odometry message over ROS
		update_ros_odom();
		odom_pub.publish(odom);

		// Some logging msgs here to ensure the node is working
		log_odom();

		ros::spinOnce();
		loop_rate.sleep();
	}
	reader.close();
}

int main(int argc, char **argv){

	// Anon is needed since multiple UAV nodes will be created
	ros::init(argc, argv, "uav", ros::init_options::AnonymousName);
	
	uav_odom uav_instance;
	uav_instance.odom_manager();
}