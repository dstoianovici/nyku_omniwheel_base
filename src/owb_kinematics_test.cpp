/*
Testing program to verify Omni Wheel Kinematics
Loads RPY positions from CSV, measures actual RPY with IMU


*/

#include <equivalent_manipulator_ik/eq_man_ik.h>
#include <stdio.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>





#define GEAR_RATIO 12/1.5
#define ROTATION 4752


int settle_flag = 0;
int mt0_e;
int mt1_e;
int mt2_e;

std::vector<float> goal_rot(3);
std::vector<float> measured_rpy(3);

std::string make_file_name(){
    time_t t = std::time(0);
    struct tm* now = localtime(&t);

    char buff[80];
    strftime(buff,80,"%F-%I-%M_RPY_Test_Data.csv",now);
    std::string of_name(buff);
    std::cout << of_name <<std::endl;

    return of_name;
}

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

    if(abs(mt0_e)<70 && abs(mt1_e)< 70 && abs(mt2_e) < 70){
        settle_flag = 1;
    }

    ROS_INFO("OW Error: (%d, %d, %d)", error_msg.data[0], error_msg.data[1], error_msg.data[2]);
}


void imu_callback(const sensor_msgs::Imu& imu_msg){
    tf2::Quaternion quat_tf;
    tf2::convert(imu_msg.orientation,quat_tf);

    tf2Scalar roll, pitch, yaw;

    tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);

    measured_rpy[0] = roll;
    measured_rpy[1] = pitch;
    measured_rpy[2] = yaw;

    //ROS_INFO("RPY: %f ,%f ,%f", roll,pitch,yaw);
}


void torsoRPY_callback(const geometry_msgs::Pose torso_quat){
    tf2Scalar roll, pitch, yaw;
    tf2::Matrix3x3 torso_rot;
    tf2::Quaternion quat_temp;

    quat_temp.setW(torso_quat.orientation.w);
    quat_temp.setX(torso_quat.orientation.x);
    quat_temp.setY(torso_quat.orientation.y);
    quat_temp.setZ(torso_quat.orientation.z);

    torso_rot.setRotation(quat_temp);

    torso_rot.getRPY(roll,pitch,yaw);

    std::cout << roll << "," << pitch << "," << yaw << std::endl;

    goal_rot[0] = roll;
    goal_rot[1] = pitch;
    goal_rot[2] = yaw;
}



std::vector<int> ik_to_motors(float gear_ration, std::vector<float> ik){
    std::vector<int> mot_out(3);
    ik.resize(3);

    for(int i = 0; i < 3;i++ ){
        mot_out[i] = -(ik[i]*(180/PI_))* GEAR_RATIO * (ROTATION/360); //Negative transmission ration
    }
    return mot_out;
}

float deg_2_rad(float deg){
    return deg*(PI_/180);
}


std::vector<std::vector<float>> read_test_data(std::string file_name){
    std::ifstream file;
    std::vector<std::vector<float>> goal_rpy_list;
    file.open(file_name.c_str(), std::ios::in);

    if(file.is_open()){
        std::string line;
        uint j = 0;
        while(std::getline(file, line)){
            std::stringstream ss(line);
            std::string rad_s;

            std::vector<float> goal_rpy(3);
            float rad_f;

            uint i = 0;
            while(std::getline(ss,rad_s,',')){
                rad_f = deg_2_rad(std::atof(rad_s.c_str()));
                goal_rpy[i] = rad_f;
                i++;
            };
            goal_rpy_list.push_back(goal_rpy);
            j++;
        }
        goal_rpy_list.resize(j);
        file.close();
    }

    else std::cout << "Could not open file"  << std::endl;

    return goal_rpy_list;
}

void print_file(std::vector<std::vector<float>> file_list){
    for(int i = 0; i < file_list.size(); i++){
        std::cout<<file_list[i][0] << "," << file_list[i][1] << "," <<  file_list[i][2] << std::endl;
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "kinematics_test_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<std_msgs::Int32MultiArray>("OWB_goal_pos", 1000);
    ros::Subscriber pos_error_sub = nh.subscribe("OWB_pos_error", 500, &error_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 500, &imu_callback);
    ros::Rate e_rate(100);


    //std::string test_file_name = "goal_pos_test_roll.csv";
    //std::string test_file_name = "roll_pos_test.csv";
    //std::string test_file_name = "pitch_pos_test.csv";
    std::string test_file_name = "yaw_pos_test.csv";


    std::string of_file_name = make_file_name(); //makes output data name based on time and date

    std::fstream data_file;
    data_file.open(of_file_name.c_str(),std::ios::out | std::ios::app);
    if(data_file.is_open()) std::cout << "Created data output file: " << of_file_name <<std::endl;
    else std::cout << "Failed to create file \n";



    std::vector<std::vector<float>> test_data; //Vector of RPY positions in degrees for testing trajectory
    test_data = read_test_data(test_file_name);




    std_msgs::Int32MultiArray goal_pos;
    goal_pos.data.resize(3);

    IK_Solver_OW ik;

    std::vector<int>  ow_mot(3);
    std::vector<float> ow_pos_rad(3);



    for(int i =0; i < test_data.size(); i++){
        goal_rot = test_data[i];

        ow_mot = ik_to_motors(GEAR_RATIO, ik.solve_ik_4q(goal_rot));
        goal_pos = create_goal_msg(ow_mot);
        goal_pub.publish(goal_pos);

        settle_flag = 0;

        while(settle_flag == 0){ //wait for motor to reach set point before sending to next setpoint
            ros::spinOnce();
        }

        //Let IMU settle
        sleep(1);
        ros::spinOnce();

        data_file<<measured_rpy[0] << "," << measured_rpy[1]<< "," << measured_rpy[2] << std::endl;
    }

    data_file.close();
    
    ros::shutdown();
    return 0;
}



