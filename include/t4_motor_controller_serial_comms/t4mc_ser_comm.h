#ifndef T4MC_SER_COMM_H
#define T4MC_SER_COMM_H

    #include <stdio.h>
    #include <stdlib.h>
    #include <cstring>
    #include <cerrno>
    #include <iostream>
    #include <fstream>
    #include <string>
    #include <vector>


    //Linux Headers
    #include <unistd.h> //unix standard function defs read() write() open
    #include <fcntl.h> // File control definitions
    #include <errno.h> //Error functions
    #include <termios.h> //POSIX Terminal Controls

    // C library headers
    #include <stdio.h>




    class Serial_Comm
    {
    public:
        Serial_Comm(std::string port_addr);
    
        void SendPacket(std::vector<int>& setpoints);


    private:
        long _baud_rate;
        char* _port_addr;
        uint _msg_size;
        int _usb_port;

        char* Create_Msg(std::vector<int>& setpoints);
        void OpenPort();


    };

#endif
