#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <quartic.h>

#define INTERVAL 2

class UAVObj
{
public:
    nav_msgs::Odometry odom;

    Eigen::Matrix4d eigen;

    UAVObj(double x_max_acc_, double y_max_acc_, double z_max_acc_);

    void set_odom(nav_msgs::Odometry msg);

    bool check_interference(UAVObj o);

    // void test(Eigen::Vector3d e1_size, Eigen::Matrix4d e1_tf, Eigen::Vector3d e2, Eigen::Matrix4d e2_tf);
private:
    // ros::NodeHandle nh_;
    // ros::Publisher cmd_vel_pub_;
    double x_max_acc, y_max_acc, z_max_acc;
    double interval_;
    int id;
    // tf::TransformListener tf_listener;
    // Eigen::Matrix4d quad[8];

    void get_future();

    Eigen::Matrix4d get_tf(const geometry_msgs::Pose::ConstPtr& pose);

    // std::vector<double> determinant(std::vector<double> matrix[4][4], int n);

    // double get_lambda(Eigen::Matrix4d a, Eigen::Matrix4d b);

};
Eigen::Matrix4d get_covariance(double x, double y, double z);
double get_future_pos(double acc, double time);
Eigen::Matrix4d tf_to_eigen(tf2::Transform tf);
bool intercept_ellipse(Eigen::Matrix4d a, Eigen::Matrix4d b);