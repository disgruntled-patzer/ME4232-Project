/*

   ROS Node to simulate a UAV in 3D space.

   Basic tf code is adopted and modified from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

   Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

   <insert license statement>

*/

#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define MAX_TRAJ 4 // Max number of CSV trajectory files
#define MAX_INITIAL_POS 5 // Max possible value of starting XYZ coordinates
#define DEBUG

// Describe Pos, Vel, Acc for a 6 DOF model (linear XYZ and angular XYZ thetas) at a certain timestep.
// Units: m, m/s, rad, rad/s.
// The positions are relative to the GCS, or 'odom' frame, the velocities are relative to the UAV, or 'base_link' frame
// Note that xth, yth and zth is angle of the UAV wrt odom origin, not the roll/pit/yaw of UAV
typedef struct {
	double x,y,z,xth,yth,zth,vx,vy,vz,v_xth,v_yth,v_zth,ax,ay,az;
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
		ros::Rate loop_rate = 1;

		void init_state();
		void access_imu_data();
		void extract_imu_data();
		void compute_odom();
		void update_ros_tf();
		void update_ros_odom();
		void log_odom();
		void odom_manager();
};

// Initialise the 6-DOF state
void uav_odom::init_state(){
	// Starting XYZ position is random
	srand(time(0));
	state.x = double(rand() % MAX_INITIAL_POS);
	state.y = double(rand() % MAX_INITIAL_POS);
	state.z = double(rand() % MAX_INITIAL_POS);
	// All other starting values are zero
	state.xth = state.yth = state.zth = 0.0;
	state.vx = state.vy = state.vz = 0.0;
	state.v_xth = state.v_yth = state.v_zth = 0.0;
	state.ax = state.ay = state.az = 0.0;
	state.current_time = state.last_time = ros::Time::now();
}

// Randomly select and open a CSV trajectory file
void uav_odom::access_imu_data(){
	std::string location;
	n.getParam("imu_data_location", location); // Location to csv files
	srand(time(0));
	int filename = rand() % MAX_TRAJ; // Filename is a random number from 0 to (MAX_TRAJ - 1)
	std::stringstream ss;
	ss << location << filename << ".csv"; // Assembly the full directory to the file
	imu_data_src = ss.str();
	#ifdef DEBUG
		ROS_INFO_STREAM("Opening file " << imu_data_src);
	#endif
	// Once filename is generated, open it
	reader.open(imu_data_src, std::fstream::in);
	if (!reader.is_open()){
		throw std::runtime_error("UAV: Could not read IMU data");
	}
	std::getline(reader, line); // Read and discard first row (which contains headers)
}

// Extract IMU data from csv file and put into the 6-DOF state
void uav_odom::extract_imu_data(){
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
}

// Update the 6-DOF odometry state for each time increment using the IMU linear accel + gyro angular vel reaidngs
void uav_odom::compute_odom(){
	state.current_time = ros::Time::now();
	double dt = (state.current_time - state.last_time).toSec();
	double dvx = 0.5*(state.ax + reading.ax)*dt;
	double dvy = 0.5*(state.ay + reading.ay)*dt;
	double dvz = 0.5*(state.az + reading.az)*dt;
	double dx = 0.5*(state.vx + (state.vx + dvx))*dt;
	double dy = 0.5*(state.vy + (state.vy + dvy))*dt;
	double dz = 0.5*(state.vz + (state.vz + dvz))*dt;
	double dxth = 0.5*(state.v_xth + reading.v_xth)*dt;
	double dyth = 0.5*(state.v_yth + reading.v_yth)*dt;
	double dzth = 0.5*(state.v_zth + reading.v_zth)*dt;
	state.x += dx;
	state.y += dy;
	state.z += dz;
	state.xth += dxth;
	state.yth += dyth;
	state.zth += dzth;
	state.vx += dvx;
	state.vy += dvy;
	state.vz += dvz;
	state.v_xth = reading.v_xth;
	state.v_yth = reading.v_yth;
	state.v_zth = reading.v_zth;
	state.ax = reading.ax;
	state.ay = reading.ay;
	state.az = reading.az;
	state.last_time = state.current_time;
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
	ROS_INFO_STREAM("Quat: [" << odom.pose.pose.orientation.x << ", " << odom.pose.pose.orientation.y << ", " << odom.pose.pose.orientation.z << ", " << odom.pose.pose.orientation.w << "]");
	ROS_INFO_STREAM("Linear Vel: [" << odom.twist.twist.linear.x << ", " << odom.twist.twist.linear.y << ", " << odom.twist.twist.linear.z << "]");
	ROS_INFO_STREAM("Ang Vel: [" << odom.twist.twist.angular.x << ", " << odom.twist.twist.angular.y << ", " << odom.twist.twist.angular.z << "]");
}

// "Main" function to broadcast tf and odom msgs to GCS and receive cmds
void uav_odom::odom_manager(){

	init_state();
	access_imu_data();

	while (ros::ok() && std::getline(reader, line)){

		// Extract imu data from the specified csv file and compute odometry
		extract_imu_data();
		compute_odom();

		// Since all odometry is 6DOF we'll need a quaternion created from x_theta, y_theta, z_theta
		odom_quat.setRPY(state.v_xth,state.v_yth,state.v_zth);

		// Publish the transform over tf
		update_ros_tf();
		odom_broadcaster.sendTransform(odom_trans);

		// Publish the odometry message over ROS
		update_ros_odom();
		odom_pub.publish(odom);

		// Some logging msgs here to ensure the node is working
		#ifdef DEBUG
			log_odom();
		#endif

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