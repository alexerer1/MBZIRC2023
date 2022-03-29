#ifndef FLIRCAMERA_HPP
#define FLIRCAMERA_HPP

// #include "componentserver.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// #include "xmlio.h"
#include "type_defines.hpp"

#include "GimbalControl.hpp"

class FlirCamera {

private:
    

    // struct xml_in *xmlflir;
    // componentservertype lmssrv;

    // void serverconnect(componentservertype *s);
    
    pose_t flir_pose;
    int num_frames = 0;
    float x_ang = 0;
    float y_ang = 0;
    float maxTemp = 0;

    GimbalControl gimbal;


public:
    

    int old_num_pose = 0;
    
    void print_data();

    int set_gimbal_angle( float angle );
    float get_gimbal_angle();

    pose_t getDirectionAndDistance(float height, float angle);
    pose_t getYZDistances(float x_distance, float angle, float heading);

    float getMaxTemp();

    pose_t getPosition(float roll, float pitch, float h, float head);

    bool init_server();

    bool getData();
    void close();
    
    
};

#endif
