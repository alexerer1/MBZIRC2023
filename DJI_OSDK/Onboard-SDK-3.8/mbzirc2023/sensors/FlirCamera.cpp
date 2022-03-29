#include "FlirCamera.hpp"
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
#include <iostream>
#include <string.h>
#include <string>
#include <cmath>

bool FlirCamera::init_server(){
 
    // *************************************
    //                 Setting up FLIR 
    
//    lmssrv.port=65433;
//    strcpy(lmssrv.host,"127.0.0.1");
//    lmssrv.config=1;
//    strcpy(lmssrv.name,"flirserver");
//    lmssrv.status=1;

//     // **************************************************
//     //  LMS server code initialization
//     //

//     /* Create endpoint */

//     flir_pose.x = 0;
//     flir_pose.y = 0;
//     flir_pose.z = 0;
//     flir_pose.valid = 0;
//     flir_pose.num_pose = 0;
    
//     if (lmssrv.config) {
//         int errno1 = 0; 
//         lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
//         if ( lmssrv.sockfd < 0 )
//         {
//             perror(strerror(errno1));
//             fprintf(stderr," Can not make  socket\n");
//             exit(errno1);
//             return 0;
//         }

//         serverconnect(&lmssrv);
    
//         xmlflir=xml_in_init(4096,32);
//         printf(" flir  xml initialized \n");
//     }  

//     if( !gimbal.init() )
//     {
//         printf("########################\n###################\n GIMBAL NOT INITIALIZED. RUN AS SUDO!!\n\n");
//         return 0;
//     }

    return 1;
    
}

// void FlirCamera::serverconnect(componentservertype *s){
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

int FlirCamera::set_gimbal_angle( float angle )
{
    if( !gimbal.set_angle(angle) )
    {
        printf("Invalid gimbal angle or pins not initilized\n");
        return 0;
    }
    return 1;
}

float FlirCamera::get_gimbal_angle()
{
    return gimbal.get_angle();
}

bool FlirCamera::getData(){
    
//     char msg[100];
// //     char empty[100];
// //     int valread = read( lmssrv.sockfd, empty, 1024);
    
//     int len = sprintf(msg, "request_angle");
    
//     send(lmssrv.sockfd,msg,len,0);
//     usleep(100);
//     char ans_buf[100];
//     int valread = read( lmssrv.sockfd, ans_buf, 1024);
// //     std::cout << ans_buf << " # !" << std::endl;
//     std::string answer(ans_buf);
    
//     if( answer.compare(0,7,"no_fire") == 0 ){
//         // printf("No fire!!!!!!!\n");
//         return 0;
//     }
//     else
//     {
        
//         int ack = 0;
        
//         sscanf(ans_buf, "%f,%f,%f,%i,%i,", &x_ang, &y_ang, &maxTemp, &num_frames,&ack);
        
//         // printf("Received: %f | %f | %i\n", x_ang, y_ang, num_frames);
//         if( ack == 77 ) return 1;
//         else return 0;
//     }

    return 1;

}

void FlirCamera::print_data()
{
    if(getData()) printf("X ang = %f deg  |  Y_ang = %f deg  | Frame = %d\n", x_ang, y_ang, num_frames);
    else std::cout << "No fire" << std::endl;
}

pose_t FlirCamera::getYZDistances(float x_distance, float angle, float heading )
{
    old_num_pose = flir_pose.num_pose;
    
    if( getData() ){
        
        flir_pose.x = x_distance;
        // flir_pose.y = -x_distance * tan( heading + x_ang*M_PI/180.0 );
        flir_pose.y = -x_distance * sin( heading - x_ang*M_PI/180.0 );
        flir_pose.z = x_distance * tan( angle - y_ang*M_PI/180.0 );
        flir_pose.th = 0;
        flir_pose.valid = 1;
        flir_pose.num_pose = num_frames;
        
    }
    else{
        flir_pose.valid = 0;
    }
    
    return flir_pose;
}

float FlirCamera::getMaxTemp()
{
    old_num_pose = flir_pose.num_pose;
    if( !getData() ) maxTemp = -1;
    return maxTemp;
}

pose_t FlirCamera::getDirectionAndDistance(float height, float angle)
{
    old_num_pose = flir_pose.num_pose;
    
    if( getData() ){
        
        //flir_pose.x = height * tan( M_PI/4.0 - y_ang*M_PI/180.0);
        flir_pose.x = height * tan( M_PI/4 - (angle*M_PI/180 + y_ang*M_PI/180.0) );
        flir_pose.y = 0;
        flir_pose.z = height;
        flir_pose.th = x_ang*M_PI/180;
        flir_pose.valid = 1;
        flir_pose.num_pose = num_frames;
        
    }
    else{
        flir_pose.valid = 0;
    }
    
    return flir_pose;
}

pose_t FlirCamera::getPosition(float roll, float pitch, float h, float head){

    old_num_pose = flir_pose.num_pose;
    
    if( getData() ){
        
        float pxPDg = 640.0/57.0;
        
        float real_x_ang = (y_ang * pxPDg)*cos(roll) - (x_ang * pxPDg)*sin(roll);
        float real_y_ang = (x_ang * pxPDg)*sin(roll) + (y_ang * pxPDg)*cos(roll);
        
        real_x_ang /= pxPDg;
        real_y_ang /= pxPDg;
        
//         float x = h * tan( M_PI/4.0 - real_y_ang*M_PI/180.0 + pitch);
        
//         float y = x * tan( -real_x_ang*M_PI/180.0 );
        
        float x = h * tan( M_PI/4.0 - real_y_ang*M_PI/180.0 + pitch);
        
        float y = x * tan( -real_x_ang*M_PI/180.0 );
        
//         printf("x_ang = %3.1f   rx_ang = %3.1f   y_ang = %3.1f   ry_ang = %3.1f   x = %3.1f   y = %3.1f    h = %3.2f   a = %3.2f\n", x_ang, real_x_ang, y_ang, real_y_ang, x, y, h, pitch*180.0/M_PI);
        
//         printf("Fire %f meters in front, %f meters sideways\n",x, y);

        float rot_x = -x*cos(head) + y*sin(head);
        float rot_y = -x*sin(head) - y*cos(head);
        
        
        flir_pose.x = rot_x;
        flir_pose.y = rot_y;
        flir_pose.z = h;
        flir_pose.valid = 1;
        flir_pose.num_pose = num_frames;
        
    }
    else{
        flir_pose.valid = 0;
    }
    
    return flir_pose;
    
}

void FlirCamera::close(){
    char msg[100];
    
    // int len = sprintf(msg, "shutdown");
    // send(lmssrv.sockfd,msg,len,0);
    // usleep(500000);
    // send(lmssrv.sockfd,msg,len,0);
    
}

