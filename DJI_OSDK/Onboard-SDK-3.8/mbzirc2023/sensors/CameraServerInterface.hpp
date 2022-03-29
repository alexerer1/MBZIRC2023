#ifndef CAMERASERVER_HPP
#define CAMERASERVER_HPP

//#include "componentserver.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include "xmlio.h"
#include "type_defines.hpp"

// The structure for storing the data send by the jetson program Joachim wrote
struct JetsonData {
    float horz_angle;
    float vert_angle;
    float distance;
    char network;
    int frame_number;
    bool valid;
};

class CameraServer {

private:
    
    int sock = 0;
    int port = 0; 
    struct sockaddr_in serv_addr;
    

    //struct xml_in *xmlflir;
    //componentservertype lmssrv;

    //void serverconnect(componentservertype *s);
    
    pose_t camera_pose;
    int num_frames = 0;

    bool Jetson = 0;

    int locType = NONE;
    
    float gimbal_angle = 0;


    pose_t returned_pose;
    
    
    
public:
    
    int old_num_pose = 0;
    
    float height = 0;
    float heading = 0;

    void set_atti( float roll, float pitch, float yaw, float height, float raw_rf );
    bool set_localizer_type(int type);
    
    int set_gimbal_angle(float angle);

    int get_localizer_type();
    

    bool init_server(bool use_jetson);
    pose_t get_camera_position();
    // void close();

    JetsonData get_Jetson_data();
    
};


#endif
