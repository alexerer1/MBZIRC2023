/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */ 
 //K-develope

//#include "LocateBuilding.hpp"
#include "GeneralCommands.hpp"
#include "componentserver.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "xmlio.h"

// OpenCV libs
/*#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/aruco.hpp"
*/

#include "CameraLocalizer.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"

// Raspicam
#include <raspicam/raspicam_cv.h> 

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

extern double laserpar[10];
extern char buf[256];
extern int len;
 extern struct xml_in *xmllaser;
extern componentservertype lmssrv;
/*! Setting up the ULMS-server 
!*/
void serverconnect(componentservertype *s){
  char buf[256];
  int len;
  s->serv_adr.sin_family = AF_INET;
  s->serv_adr.sin_port= htons(s->port);
  s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
  printf("port %d host %s \n",s->port,s->host);
  if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
    printf(" connected to %s  \n",s->name);
    len=sprintf(buf,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    send(s->sockfd,buf,len,0);
    len=sprintf(buf,"mrc version=\"1.00\" >\n");
    send(s->sockfd,buf,len,0);
    if (fcntl(s->sockfd,F_SETFL,O_NONBLOCK) == -1) {
          fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on %s fd \n",s->name);
    }
  }
  else{
      printf("Not connected to %s  %d \n",s->name,s->connected);   
  }
}



/*! main
 *
 */
int
main(int argc, char** argv)
{
// *************************************
//                 Setting up Lidar 
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   lmssrv.config=1;
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;

// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   if (lmssrv.config) {
      int errno1 = 0; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno1));
    fprintf(stderr," Can not make  socket\n");
    exit(errno1);
   }

   serverconnect(&lmssrv);

   xmllaser=xml_in_init(4096,32);
   printf(" laserserver xml initialized \n");

}  


  // Display interactive prompt
	std::cout
	<< "| Available commands:                                            |"
    << std::endl;
	std::cout << "| Press [a]: Test Rangefinder            |" << std::endl;
  std::cout << "| Press [b]: Test laser scanner            |" << std::endl;
  std::cout << "| Press [c]: Test camera           |" << std::endl;
  std::cout << "| Press [d]: ??            |" << std::endl;
	char inputChar;
	std::cin >> inputChar;


  char i2c_enabled = 0;

  raspicam::RaspiCam_Cv cap;
  cv::Mat new_frame;

  CameraLocalizer downCamera(320,240);//(640,480);

  switch(inputChar){
    case 'a':
      initializeI2C( &i2c_enabled );
      if( &i2c_enabled ){
        for(int i = 0; i < 5 ; i++ ) {
          float rangefinder = getHeight( &i2c_enabled );
          std::cout << "Rangefinder = " << rangefinder << " m\n";
          sleep(1);
        }
      }
      else {
        std::cout << "Rangefinder not connected\n";
      }

      break;
    case 'b':
      len=sprintf(buf,"<scanpush cmd='zoneobst total=100'/>\n");
      send(lmssrv.sockfd,buf,len,0);
      sleep(1);
      getLidarData();
      std::cout << "zoneobst data = ";
      for(int i = 0; i < 10; i++ ){
        std::cout << laserpar[i] << "  ";
      }
      std::cout << std::endl;
      break;
    case 'c':

      if( init_cam( &cap ) ) std::cout << "Camera on\n";
      else break;
      
      while( 1 ){
        cap.grab();
        cap.retrieve(new_frame);

        //cv::imshow("Drone camera view",new_frame);

        //if ( cv::waitKey(50) == 'q' ) break;
      }
      cv::destroyAllWindows();
      cap.release();

      break;
    case 'd':
      if(downCamera.init_camera())
        std::cout << "Camera on" << std::endl;
      else{
        std::cout << "Camera not oppened\n";
        break;
      }
      
      downCamera.cameraMainLoop();
      
      std::cout << "Camera running waiting 10 seconds" << std::endl;
      
      sleep(100);
      
      downCamera.stop_camera();
      
      break;
    default:
      break;
  }

  if( !shutdown(lmssrv.sockfd,SHUT_RDWR) )
    std::cout << "Socket closed\n";

  return 0;
}
