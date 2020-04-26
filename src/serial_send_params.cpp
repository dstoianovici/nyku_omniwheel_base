#include <ros/ros.h>
#include "t4_motor_controller_serial_comms/t4mc_ser_comm.h"
#include "equivalent_manipulator_ik/eq_man_ik.h"
#include <stdio.h>
#include "std_msgs/Int32MultiArray.h"


#define GEAR_RATIO 12/1.5
#define ROTATION 4752



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
    
    ROS_INFO("OW Error: (%d, %d, %d)", error_msg.data[0], error_msg.data[1], error_msg.data[2]);

}


std::vector<int> ik_to_motors(float gear_ration, std::vector<float> ik){
    std::vector<int> mot_out(3);
    ik.resize(3);

    for(int i = 0; i < 3;i++ ){
        mot_out[i] = -(ik[i]*(180/PI))* GEAR_RATIO * (ROTATION/360); //Negative transmission ration
    }
    return mot_out;
}

float deg_2_rad(int deg){
    return deg*(PI/180);
}


int main(int argc, char* argv[]){

    ros::init(argc, argv, "OWB_IK_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<std_msgs::Int32MultiArray>("OWB_goal_pos", 1000);
    ros::Subscriber pos_error_sub = nh.subscribe("OWB_pos_error", 1000, &error_callback);


    std_msgs::Int32MultiArray goal_pos;
    goal_pos.data.resize(3);


    IK_Solver_OW ik;

    std::vector<float> goal_rot(3);
    std::vector<int>  ow_mot(3);
    std::vector<float> ow_pos_rad(3);

    goal_rot[0] = deg_2_rad(0); //about x global
    goal_rot[1] = deg_2_rad(0); //about y
    goal_rot[2] = deg_2_rad(179); //about z

    ow_pos_rad = ik.solve_ik_4q(goal_rot);
    ow_mot = ik_to_motors(GEAR_RATIO, ow_pos_rad);
    goal_pos = create_goal_msg(ow_mot);


    goal_pub.publish(goal_pos);


    ros::spin();
    

    return 0;
}
