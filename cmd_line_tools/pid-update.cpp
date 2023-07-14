#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <unistd.h>

#define CAN_PACKET_SET_PID 253U

using namespace std;

template<typename T>
void SwapEndian(T& val)
{
    union U {
        T val;
        uint8_t raw[sizeof(T)];
    } src, dst;

    src.val = val;
    reverse_copy(reinterpret_cast<uint8_t*>(&src.val),
                      reinterpret_cast<uint8_t*>(&src.val) + sizeof(T),
                      dst.raw);
    val = dst.val;
}

int main(int argc, char **argv)
{
    //user input, file with vesc_id, ctrl_type, p, i, d
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " /file/with/pid.data" << endl;
        cerr << "Example file created, pid-example.data" << endl;
        ofstream file("pid-example.data");
        file << "#VESC_ID CTRL_TYPE P I D -> Single space deliminated" << endl;
        file << "101 position 0.1000 0.0100 0.0010" << endl;
        file << "102 velocity 0.3000 0.0100 0.0010" << endl;
        return EXIT_FAILURE;
    }

    //parse file
    uint8_t vesc_id;
    uint32_t pid[3];
    uint16_t ctrl_type, p_i_d_s;
    struct can_frame frame;
    int sckt;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sckt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can0" );
    ioctl(sckt, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    memset(&frame, 0, sizeof(frame));

    if (bind(sckt, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        cerr << "Error in socket bind" << endl;
    }

    ifstream file(argv[1]);
    if (!file) {
        cerr << "Error: could not open file " << argv[1] << endl;
        return EXIT_FAILURE;
    }

    string token, line;
    while (file) {
        //get vesc data
        vector<string> tokens;
        getline(file, line);
        if (strchr(line.c_str(), '#')) {
            continue;
        }
        istringstream ss(line);
        while (std::getline(ss, token, ' ')) {
            tokens.push_back(token);
        }
        if(tokens.size() != 5) { continue; }

        vesc_id = atoi(tokens[0].c_str());

        if (tokens[1] == "position") {
            ctrl_type = 0;
        } else if (tokens[1] == "velocity") {
            ctrl_type = 1;
        } else if (tokens[1] == "current") {
            ctrl_type = 2;
        } else {
            cerr << "Error: unknown control type " << tokens[1] << endl;
            return EXIT_FAILURE;
        }
        
        pid[0] = atof(tokens[2].c_str()) * 1E6;
        pid[1] = atof(tokens[3].c_str()) * 1E6;
        pid[2] = atof(tokens[4].c_str()) * 1E6;

        //compile and send can commands;
        frame.can_id = ((vesc_id | ((uint32_t)CAN_PACKET_SET_PID << 8)) | CAN_EFF_FLAG);
        frame.can_dlc = 8;
        frame.__pad = 0;
        cout << "vesc id " << (int)vesc_id << " ctrl type " << ctrl_type << " p " << pid[0] << " i " << pid[1] << " d " << pid[2] << endl;

        SwapEndian(pid[0]);
        SwapEndian(pid[1]);
        SwapEndian(pid[2]);
        SwapEndian(ctrl_type);


        uint8_t data[8];
        for (int i = 1; i < 4; i++) {
            p_i_d_s = i;
            SwapEndian(p_i_d_s);
            memcpy(data, &pid[i-1], 4);
           // printf("adding data %u\n", pid[i-1]);
            memcpy(data+4, &ctrl_type, 2);
            memcpy(data+6, &p_i_d_s, 2);
            memcpy(frame.data, data, 8);

            int nbytes = write(sckt, &frame, sizeof(struct can_frame));
            if (nbytes != sizeof(struct can_frame)) {
                cerr << "not all bytes writtes, nbytes: " << nbytes << endl;
            }
        }
        //tell it to save the data
        p_i_d_s = 4;
        SwapEndian(p_i_d_s);

        memset(data, 0, 8);
        memcpy(data+4, &ctrl_type, 2);
        memcpy(data+6, &p_i_d_s, 2);
        memcpy(frame.data, data, 8);
        int nbytes = write(sckt, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            cerr << "not all bytes writtes, nbytes: " << nbytes << endl;
        }
    }

    return EXIT_SUCCESS;
}
