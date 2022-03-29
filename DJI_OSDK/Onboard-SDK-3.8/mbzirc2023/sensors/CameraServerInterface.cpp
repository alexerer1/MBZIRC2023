#include "CameraServerInterface.hpp"
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
#include <iostream>
#include <string.h>
#include <string>
#include <cmath>
#include <fcntl.h>

/************************************************
 *
 *   Function for initializing the cam server interface
 *   The server needs to be running
 *   Pass 0 to use localhost cameraserver and 0
 *   to use the jetson address
 *
 *************************************************/
bool CameraServer::init_server(bool use_jetson)
{

    // Set the correct address
    //     lmssrv.port=7740;
    //     if( use_jetson )
    //     {
    //         lmssrv.port=65000;
    //         strcpy(lmssrv.host,"192.168.0.35");
    //         Jetson = use_jetson;
    //     }
    //     else
    //     {
    //         lmssrv.port=7740;
    //         strcpy(lmssrv.host,"127.0.0.1");
    //     }

    //    lmssrv.config=1;
    //    strcpy(lmssrv.name,"CameraServer");
    //    lmssrv.status=1;

    // **************************************************
    //  LMS server code initialization
    //

    /* Create endpoint */

    returned_pose.x = 0;
    returned_pose.y = 0;
    returned_pose.z = 0;
    returned_pose.valid = 0;
    returned_pose.num_pose = 0;

    // Connect to the socket
    // if (lmssrv.config) {
    //     int errno1 = 0;
    //     lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    //     if ( lmssrv.sockfd < 0 )
    //     {
    //         perror(strerror(errno1));
    //         fprintf(stderr," Can not make  socket\n");
    //         exit(errno1);
    //         return 0;
    //     }

    //     serverconnect(&lmssrv);

    //     xmlflir=xml_in_init(4096,32);
    //     printf(" CameraServer  xml initialized \n");
    // }

    // return 1;

    port = 7740;
    char ip_string[16];
    if (use_jetson)
    {
        port = 65000;
        strcpy(ip_string, "192.168.0.35");
        Jetson = use_jetson;
    }
    else
    {
        port = 7740;
        strcpy(ip_string, "127.0.0.1");
    }

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) == -1)
    {
        printf("\n startserver: Unable to set flag O_NONBLOCKn");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, ip_string, &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed (Becuase of NONBLOCKIGN set) - disregard\n");
        // return -1;
    }

    return 1;
}

/************************************************
 *
 *   Connect to a server
 *
 *************************************************/
// void CameraServer::serverconnect(componentservertype *s){
//   char buf[256];
//   int len;
//   s->serv_adr.sin_family = AF_INET;
//   s->serv_adr.sin_port= htons(s->port);
//   s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
//   printf("port %d host %s \n",s->port,s->host);
//   if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
//     printf(" connected to %s  \n",s->name);
// //     len=sprintf(buf,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
// //     send(s->sockfd,buf,len,0);
// //     len=sprintf(buf,"mrc version=\"1.00\" >\n");
// //     send(s->sockfd,buf,len,0);
//     if (fcntl(s->sockfd,F_SETFL,O_NONBLOCK) == -1) {
//           fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on %s fd \n",s->name);
//     }
//   }
//   else{
//       printf("Not connected to %s  %d \n",s->name,s->connected);
//   }
// }

/************************************************
 *
 *   Gets data from server
 *   Returns the data read from the server
 *
 *************************************************/
pose_t CameraServer::get_camera_position()
{
    // Send a request
    char msg[100];
    int len = sprintf(msg, "get_pose");
    // send( lmssrv.sockfd, msg, len, 0 );
    send(sock, msg, len, 0);
    usleep(100);
    // Read the response
    char ans_buf[100];
    int valread = read(sock, ans_buf, 1024);
    std::string answer(ans_buf);
    // Check if the answer is valid
    if (answer.compare(0, 4, "pose") != 0)
    {
        returned_pose.valid = 0;
    }
    else
    {
        // std::string end = "end";
        // size_t found = answer.find(end);
        // std::cout << "Found = " << found << std::endl;
        // char real_ans[found+3];
        // for( int i = 0; i<found+3;i++) real_ans[i] = ans_buf[i];
        // std::cout << "R = " << real_ans << "|||" <<std::endl;
        float x, y, z, th;
        sscanf(ans_buf, "pose: %f %f %f %f %d %d end", &x, &y, &z, &th, &returned_pose.num_pose, &returned_pose.valid);

        // Do the correct convesion from camera server data to positions
        if (locType == FIND_BOX)
        {
            // std::cout << "BOX FOUND" << height << std::endl;
            float x_t = y * (height - 0.32) / 630.0;
            float y_t = x * (height - 0.32) / 630.0;

            x = x_t * cos(heading) - y_t * sin(heading);
            y = x_t * sin(heading) + y_t * cos(heading);
            // x = x_t;
            // y = y_t;
        }

        // Package the data
        returned_pose.x = x;
        returned_pose.y = y;
        returned_pose.z = z;
        returned_pose.th = -th;
        if (locType == FIND_BALLOON)
        {
            returned_pose.th = th;
        }

        // printf("Received %f : %f | %f | %f | %f | %d | %d\n", height, returned_pose.x, returned_pose.y, returned_pose.z, returned_pose.th, returned_pose.num_pose, returned_pose.valid);

        camera_pose = returned_pose;
    }
    return returned_pose;
}

/************************************************
 *
 *   Send the attitude of the drone to the server
 *
 *************************************************/
void CameraServer::set_atti(float roll, float pitch, float yaw, float height, float raw_rf)
{
    char msg[100];
    int len = sprintf(msg, "set_atti: %.5f %.5f %.5f %.4f %.4f end\n", roll, pitch, yaw, height, raw_rf);
    // send( lmssrv.sockfd, "X", 1, 0);
    // usleep(50);
    send(sock, msg, len, 0);
}

/************************************************
 *
 *   Return locType
 *
 *************************************************/
int CameraServer::get_localizer_type() { return locType; }

/************************************************
 *
 *   Set the localizer type. Gets and acknowlegdement
 *   from the server
 *
 *************************************************/
bool CameraServer::set_localizer_type(int type)
{
    char msg[100];
    char ans_buf[100];
    int answer;

    // Send request

    int len = sprintf(msg, "set_loc_type: %d end", type);

    send(sock, msg, len, 0);
    // Read if valid and set correct
    int valread = read(sock, ans_buf, 1024);
    sscanf(ans_buf, "loc_set %d end", &answer);
    if (answer != -1)
    {
        locType = type;
        return 1;
    }
    else
        return 0;
}

/************************************************
 *
 *   Request and read data from Jetson program
 *
 *
 *************************************************/
JetsonData CameraServer::get_Jetson_data()
{

    char msg[100];

    JetsonData return_angles;

    int len = sprintf(msg, "request_data");

    // Request data
    send(sock, msg, len, 0);
    usleep(100);
    char ans_buf[100];
    int valread = read(sock, ans_buf, 1024);
    std::string answer(ans_buf);

    // Read the returned data
    float x, y, distance;
    int frame, valid_frame;
    char type;
    // sscanf(ans_buf, "pose: %f %f %f %f %d %d end", &x, &y, &z, &th, &returned_pose.num_pose, &returned_pose.valid);
    sscanf(ans_buf, "%f,%f,%f,%c,%d,%dEND", &x, &y, &distance, &type, &frame, &valid_frame);

    return_angles.horz_angle = x;
    return_angles.vert_angle = y;
    return_angles.distance = distance;
    return_angles.network = type;
    return_angles.frame_number = frame;
    return_angles.valid = valid_frame;

    return return_angles;
}

/************************************************
 *
 *   Set the angle of the gimbal. Return angle from the
 *   server is checked to see if it was succesful
 *
 *************************************************/
int CameraServer::set_gimbal_angle(float angle)
{
    char msg[100];
    char ans_buf[100];
    int answer;
    // Send a gimbal angle request
    int len = sprintf(msg, "set_angle: %f end", angle);
    send(sock, msg, len, 0);
    usleep(1000);

    // Check if it was valid
    int valread = read(sock, ans_buf, 1024);
    // printf("Answer  =  %s\n",ans_buf);
    sscanf(ans_buf, "angle_set %d end", &answer);
    if (answer == 1)
    {
        return 1;
        gimbal_angle = angle;
    }
    else
        return 0;
}