#include <ros/ros.h>
#include <equivalent_manipulator_ik/eq_man_ik.h>
#include <stdio.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



#define GEAR_RATIO 12/1.5
#define ROTATION 4752

int settle_flag = 0;
int mt0_e;
int mt1_e;
int mt2_e;

std::vector<float> goal_rot(3);


std_msgs::Int32MultiArray create_goal_msg(std::vector<int> mot_pos){
    std_msgs::Int32MultiArray goal_msg;
    goal_msg.data.resize(3);
    mot_pos.resize(3);

    goal_msg.data[0] = mot_pos[0];
    goal_msg.data[1] = mot_pos[1];
    goal_msg.data[2] = mot_pos[2];

    return goal_msg;
}


void error_callback(const std_msgs::Int32MultiArray& error_msg){

    mt0_e = error_msg.data[0];
    mt1_e = error_msg.data[1];
    mt2_e = error_msg.data[2];

    if(mt0_e==0 && mt1_e == 0 && mt2_e == 0){
        settle_flag = 1;
    }

    ROS_INFO("OW Error: (%d, %d, %d)", error_msg.data[0], error_msg.data[1], error_msg.data[2]);
}


void imu_callback(const sensor_msgs::Imu& imu_msg){
    tf2::Quaternion quat_tf;
    tf2::convert(imu_msg.orientation,quat_tf);

    tf2Scalar roll, pitch, yaw;

    tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);

    goal_rot[0] = roll;
    goal_rot[1] = pitch;
    goal_rot[2] = yaw;

    //ROS_INFO("RPY: %f ,%f ,%f", roll,pitch,yaw);
}



std::vector<int> ik_to_motors(float gear_ration, std::vector<float> ik){
    std::vector<int> mot_out(3);
    ik.resize(3);

    for(int i = 0; i < 3;i++ ){
        mot_out[i] = -(ik[i]*(180/PI_))* GEAR_RATIO * (ROTATION/360); //Negative transmission ration
    }
    return mot_out;
}

float deg_2_rad(int deg){
    return deg*(PI_/180);
}


int main(int argc, char* argv[]){

    ros::init(argc, argv, "OWB_IK_node_imu");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<std_msgs::Int32MultiArray>("OWB_goal_pos", 1000);
    ros::Subscriber pos_error_sub = nh.subscribe("OWB_pos_error", 500, &error_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 500, &imu_callback);
    ros::Rate e_rate(100);

    std_msgs::Int32MultiArray goal_pos;
    goal_pos.data.resize(3);

    IK_Solver_OW ik;

    std::vector<int>  ow_mot(3);
    std::vector<float> ow_pos_rad(3);
   

    while(ros::ok()){
        ow_mot = ik_to_motors(GEAR_RATIO, ik.solve_ik_4q(goal_rot));
        goal_pos = create_goal_msg(ow_mot);
        goal_pub.publish(goal_pos);
        ROS_INFO("Goal Pub: %d, %d, %d", goal_pos.data[0],goal_pos.data[1],goal_pos.data[2]);
        ros::spinOnce();
        e_rate.sleep();
    }

    ros::shutdown();
    return 0;
}



