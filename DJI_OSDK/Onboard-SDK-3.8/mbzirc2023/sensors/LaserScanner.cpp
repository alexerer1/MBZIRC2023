#include "LaserScanner.hpp"
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
#include <iostream>
#include <cmath>
bool LaserScanner::init_laser(){
 
    // *************************************
    //                 Setting up Lidar 
    
//    lmssrv.port=24919;
//    strcpy(lmssrv.host,"127.0.0.1");
//    lmssrv.config=1;
//    strcpy(lmssrv.name,"laserserver");
//    lmssrv.status=1;

//     // **************************************************
//     //  LMS server code initialization
//     //

//     /* Create endpoint */

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
    
//         xmllaser=xml_in_init(4096,32);
//         printf(" laserserver xml initialized \n");
//     }  

//     usleep(100000);

//     getPosition(0);
//     getPosition(0);

    return 1;
    
}

// void LaserScanner::serverconnect(componentservertype *s){
//   char buf[256];
//   int len;
//   s->serv_adr.sin_family = AF_INET;
//   s->serv_adr.sin_port= htons(s->port);
//   s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
//   printf("port %d host %s \n",s->port,s->host);
//   if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
//     printf(" connected to %s  \n",s->name);
//     len=sprintf(buf,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
//     send(s->sockfd,buf,len,0);
//     len=sprintf(buf,"mrc version=\"1.00\" >\n");
//     send(s->sockfd,buf,len,0);
//     if (fcntl(s->sockfd,F_SETFL,O_NONBLOCK) == -1) {
//           fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on %s fd \n",s->name);
//     }
//   }
//   else{
//       printf("Not connected to %s  %d \n",s->name,s->connected);   
//   }
// }

void LaserScanner::getData(){

	// laserpar[0]=-1;
	// while(laserpar[0]==-1)
	// {
	// 	while ( (xml_in_fd(xmllaser,lmssrv.sockfd) > 0 ) )
	// 	// printf("xmlprica");
	// 	xml_proca(xmllaser);
	// 	usleep(50);
	// }

}

pose_t LaserScanner::getPosition(float pitch)
{
    // pose_t position;
    // position.valid = 0;
    // position.num_pose = -1;

    // // len=sprintf(buf,"<scanpush cmd='matriceScan'/>\n"); //  total=100
    // len=sprintf(buf,"<matriceScan/>\n"); //  total=100

    // send(lmssrv.sockfd,buf,len,0);

    // usleep(100);
    // // getData();
    // char ans_buf[300];
    // int valread = read( lmssrv.sockfd, ans_buf, 1024);
    // // std::cout << ans_buf << " # !" << std::endl;
    // std::string answer(ans_buf);
    
    // if( answer.compare(0,8,"<laser l0") == 0 ){
    //     printf("invalud laser!!!!!!!\n");
    //     return position;
    // }
    // int l0, l1;
    // float l2, l3, l4, l5, l6, l7, l8; 
    // sscanf(ans_buf, "<laser l0=\"%d\"  l1=\"%d\" l2=\"%f\" l3=\"%f\" l4=\"%f\" l5=\"%f\" l6=\"%f\" l7=\"%f\" l8=\"%f\" />\n",
    //     &l0, &l1, &l2, &l3, &l4, &l5, &l6, &l7, &l8 );
    // // laserpar[0] = l0;
    // // laserpar[1] = l1;
    // // laserpar[2] = l2;
    // // laserpar[3] = l3;
    // // laserpar[4] = l4;

    // // std::cout << l0 << " | " << l1 << " | " << l2 <<  " | " << l3 << " | " << l4 << std::endl;

    // // position.num_pose = laserpar[0];

    // // if( laserpar[2] == 2.2 )
    // // {
        
    // //     position.x = cos(pitch) * laserpar[4];
    // //     position.th = -laserpar[3] * M_PI/180;
    // //     position.valid = 1;
    // // }

    // if( (0.001 > (l2 - 2.2)) )
    // {
    //     position.x = -cos(pitch) * l4;
    //     position.th = -l3 * M_PI/180;
    //     position.num_pose = l0;
    //     position.valid = 1;
    // }

    // return position;
}

// void LaserScanner::xml_proca(struct xml_in *x){

    // while(1){
    //     // std::cout << "proca\n";
    // switch (xml_in_nibble(x)) {
    //     case XML_IN_NONE:
    //         return;
    //     case XML_IN_TAG_START:
    //         #if (0)
    //         {int i;
    //         double a;
    //             printf("start tag: %s, %d attributes\n", x->a, x->n);
    //             for(i=0;i<x->n;i++){
    //             printf("  %s    %s  \n",x->attr[i].name,x->attr[i].value);
    //             }
    //         }
    //         #endif
    //             if (strcmp("laser",x->a)==0){
    //             int i,ix;
    //             for (i=0;i< x->n;i++){
    //                 ix=atoi(x->attr[i].name+1);
    //                 if (ix >-1 && ix < 10)
    //                     laserpar[ix]=atof(x->attr[i].value);
    //             }   
    //         }
            
    //         break;
    //     case XML_IN_TAG_END:
    //         //printf("end tag: %s\n", x->a);
    //         break;
    //     case XML_IN_TEXT:
    //         //printf("text: %d bytes\n  \"", x->n);
    //         //fwrite(x->a, 1, x->n, stdout);
    //         //printf("\"\n");
    //         break;
    //     }   
    //     usleep(10);
    // } 
// }  


void LaserScanner::test_zoneobst(){
// 	std::cout << "zoneobst start \n";
//     len=sprintf(buf,"<scanpush cmd='matriceScan'/>\n"); //  total=100
// std::cout << "lzoneobst sprintf\n";
//     send(lmssrv.sockfd,buf,len,0);
// std::cout << "lzoneobst send\n";
//     sleep(1);
//     getData();
// std::cout << "lzoneobst getData\n";
    
//     std::cout << "zoneobst data  = ";
//     for(int i = 0; i < 10; i++ ){
//         std::cout << laserpar[i] << "  ";
//     }
//     std::cout << std::endl;
}


double* LaserScanner::get_laserdata(){
	// test_zoneobst();
	// return &laserpar[0];
}
