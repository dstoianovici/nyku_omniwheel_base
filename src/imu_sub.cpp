/*
Testing program to verify Omni Wheel Kinematics
Loads RPY positions from CSV, measures actual RPY with IMU


*/

#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>


void imu_callback(const sensor_msgs::Imu& imu_msg){
    tf2::Quaternion quat_tf;
    tf2::convert(imu_msg.orientation,quat_tf);

    tf2Scalar roll, pitch, yaw;

    tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);

    ROS_INFO("RPY: %f ,%f ,%f", roll,pitch,yaw);
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "imu_sub");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("imu", 500, &imu_callback);
    ros::Rate e_rate(100);


    while(ros::ok()){
        ros::spinOnce();
    }
    
    ros::shutdown();
    return 0;
}



