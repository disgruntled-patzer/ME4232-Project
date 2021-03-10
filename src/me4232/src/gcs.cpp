/*

ROS Node to simulate a Ground Station

Copyright (C) 2021, Lau Yan Han and Niu Xinyuan, National University of Singapore

<insert license statement>

*/

#include <gcs.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "gcs");

    ros::NodeHandle n;
    sub = n.subscribe("uav/odom", 10, gcs_odo_callback);
    // // tf_pub = n.advertise<geometry_msgs::TransformStamped>("uav/tf_stamped", 100);
    // UAVObj a(2., 2., 1.);
    // nav_msgs::Odometry msg;
    // msg.pose.pose.position.x = 0;
    // msg.pose.pose.position.y = 0;
    // msg.pose.pose.position.z = 0;
    // msg.pose.pose.orientation.w = 1;
    // msg.pose.pose.orientation.x = 0;
    // msg.pose.pose.orientation.y = 0;
    // msg.pose.pose.orientation.z = 0;
    // msg.twist.twist.linear.x = 0;
    // msg.twist.twist.linear.y = 0;
    // msg.twist.twist.linear.z = 0;
    // msg.twist.twist.angular.x = 0;
    // msg.twist.twist.angular.y = 0;
    // msg.twist.twist.angular.z = 0;
    // a.set_odom(msg);
    // UAVObj b(2., 2., 1.);
    // nav_msgs::Odometry msg_2;
    // msg_2.pose.pose.position.x = 1.9;
    // msg_2.pose.pose.position.y = 0;
    // msg_2.pose.pose.position.z = 0;
    // msg_2.pose.pose.orientation.w = 1;
    // msg_2.pose.pose.orientation.x = 0;
    // msg_2.pose.pose.orientation.y = 0;
    // msg_2.pose.pose.orientation.z = 0;
    // msg_2.twist.twist.linear.x = 0;
    // msg_2.twist.twist.linear.y = 0;
    // msg_2.twist.twist.linear.z = 0;
    // msg_2.twist.twist.angular.x = 0;
    // msg_2.twist.twist.angular.y = 0;
    // msg_2.twist.twist.angular.z = 0;
    // b.set_odom(msg_2);
    // bool result = a.check_interference(b);
    // std::cout << "Might hit: " << result << std::endl;
    ros::spin();
    
    return 0;
}


void gcs_odo_callback(const nav_msgs::Odometry::ConstPtr& msg){
    int idx = stoi(msg->child_frame_id);
    // std::cout << "Recieved from " << idx << std::endl;
    if (idx > UAVs.size()){
        UAVObj uav(2., 2., 1.);
        UAVs.push_back(uav);
    }
    UAVs[idx - 1].set_odom(*msg);
    bool interference[UAVs.size()];
    for (int i = 0; i < UAVs.size(); i++) {
        if (i == idx - 1) {
            interference[i] = false;
            continue;
        }
        bool interference = UAVs[idx - 1].check_interference(UAVs[i]);
        if (interference) {
            std::cout << "Interference between " << i + 1 << " and " << idx << std::endl;
        }
        else {
            std::cout << "No interference between " << i + 1 << " and " << idx << std::endl;
        }
    }
}
