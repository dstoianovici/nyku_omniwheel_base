#include "ros/ros.h"  
#include "t4_motor_controller_serial_comms/t4mc_ser_comm.h"
#include "equivalent_manipulator_ik/eq_man_ik.h"
#include <stdio.h>




#define MSG_SIZE 19 //sum of 4 char long angles and delimiters
#define GEAR_RATIO 12/1.5
#define ROTATION 4752

//using namespace std;

// void write2port(int ser_port, string message){
//   unsigned char msg_arr[MSG_SIZE];
//   write(ser_port, message, sizeof(msg_arr));
// }

std::vector<int> ik_to_motors(float gear_ration, std::vector<float> ik){
    std::vector<int> mot_out(3);
    ik.resize(3);

    for(int i = 0; i < 3;i++ ){
        mot_out[i] = -(ik[i]*(180/PI))*gear_ration * (ROTATION/360); //Negative transmission ration
    }
    return mot_out;
}

float deg_2_rad(int deg){
    return deg*(PI/180);
}


int main(){

    std::string port_addr = "/dev/ttyACM0";

    Serial_Comm com(port_addr);

    IK_Solver_OW ik;

    std::vector<float> goal_rot(3);
    std::vector<int> num(3);
    std::vector<float> ow_pos2(3);



    goal_rot[0] = deg_2_rad(15); //about x global
    goal_rot[1] = deg_2_rad(0); //about y
    goal_rot[2] = deg_2_rad(0); //about z

    // ow_pos = ik.solve_ik_2q(goal_rot);
    ow_pos2 = ik.solve_ik_4q(goal_rot);
    num = ik_to_motors(GEAR_RATIO, ow_pos2);

    // std::cout<<"2Quad"<<std::endl;
    // std::cout<<ow_pos[0] <<std::endl;
    // std::cout<<ow_pos[1] <<std::endl;
    // std::cout<<ow_pos[2] <<std::endl;

    std::cout<<"4Quad"<<std::endl;
    std::cout<<ow_pos2[0] <<std::endl;
    std::cout<<ow_pos2[1] <<std::endl;
    std::cout<<ow_pos2[2] <<std::endl;
    

    std::cout<<"OW Messages"<<std::endl;
    std::cout<<num[0] <<std::endl;
    std::cout<<num[1] <<std::endl;
    std::cout<<num[2] <<std::endl;
    
  
    com.SendPacket(num);
    //sleep(2);

    goal_rot[0] = deg_2_rad(0); //about x global
    goal_rot[1] = deg_2_rad(0); //about y
    goal_rot[2] = deg_2_rad(0); //about z
    ow_pos2 = ik.solve_ik_4q(goal_rot);
    num = ik_to_motors(GEAR_RATIO, ow_pos2);
    com.SendPacket(num);
    //sleep(2);

    goal_rot[0] = deg_2_rad(-15); //about x global
    goal_rot[1] = deg_2_rad(0); //about y
    goal_rot[2] = deg_2_rad(0); //about z
    ow_pos2 = ik.solve_ik_4q(goal_rot);
    num = ik_to_motors(GEAR_RATIO, ow_pos2);
    com.SendPacket(num);
    //sleep(2);

    goal_rot[0] = deg_2_rad(0); //about x global
    goal_rot[1] = deg_2_rad(0); //about y
    goal_rot[2] = deg_2_rad(0); //about z
    ow_pos2 = ik.solve_ik_4q(goal_rot);
    num = ik_to_motors(GEAR_RATIO, ow_pos2);
    com.SendPacket(num);


    // sleep(3);
    // num = {0,0,0};
    // com.SendPacket(num);

    // while(1){
    //     sleep(2);
    //     num = {4000,4000,4000};
    //     com.SendPacket(num);

    //     sleep(2);
    //     num = {0,0,0};
    //     com.SendPacket(num);

    //     sleep(2);
    //     num = {-4000,-4000,4000};
    //     com.SendPacket(num);
    // }




    std::cout<<"routine complete"<<std::endl;

    // return 0;
}
