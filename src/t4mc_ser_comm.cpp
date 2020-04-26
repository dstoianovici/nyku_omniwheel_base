#include "t4_motor_controller_serial_comms/t4mc_ser_comm.h"


///////Constructor/////////////
Serial_Comm::Serial_Comm(std::string port_addr){
    int size = port_addr.size();
    _port_addr = (char*)malloc(size);
    strcpy(_port_addr,port_addr.c_str());

    //std::cout<<"Created SerialComm object of size " << size << " and it is " <<_port_addr << std::endl;

}





//////////Methods////////////
void Serial_Comm::SendPacket(std::vector<int>& setpoints){
    Serial_Comm::OpenPort();
    char buff[7];
    char* msg = Serial_Comm::Create_Msg(setpoints);

    write(_usb_port,msg,_msg_size);

    std::cout<<buff<<std::endl;

    close(_usb_port);
}



void Serial_Comm::OpenPort(){
    _usb_port = open(_port_addr, O_RDWR);
    struct termios tty;
    struct termios tty_old;

    if ( tcgetattr (_usb_port, &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << std::strerror(errno) << std::endl;
    }

    tty_old = tty;
    cfsetospeed (&tty, (speed_t)B115200); //Hard set baud rate for now
    cfsetispeed (&tty, (speed_t)B115200);
    tty.c_cflag &= ~PARENB;
}



char* Serial_Comm::Create_Msg(std::vector<int>& setpoints){
    std::string msg_buff;
        
    for(int i = 0; i < setpoints.size(); i++){
        msg_buff.append(std::to_string(setpoints[i]));
        if(i != setpoints.size() - 1){
            msg_buff.append(","); //add comma delimiter
        }
    }

    _msg_size = msg_buff.size();
    char* msg = (char*)malloc(_msg_size);

    strcpy(msg,msg_buff.c_str());

    return msg;
}