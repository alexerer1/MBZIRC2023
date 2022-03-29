
# include <stdio.h> 

// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"
# include <opencv2/aruco.hpp> //added

# include <sys/time.h>

#include <math.h>
#include "rotations.hpp"

#include "marker_follow.hpp"

using namespace cv;
using namespace std;

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/***************************************
*
* Captures a picture and estimates the
* position of the drone in the global
* coordinate system
*
****************************************/
//struct pose_t find_pos(raspicam::RaspiCam_Cv *cap, double roll, double pitch, bool drone_view)
//pose_t find_pos(cv::Mat *image, ar_map * global_map, double roll, double pitch)
pose_t find_marker(cv::Mat * image, bool draw_markers)
{

	//Mat src;
	Mat R33 = Mat::eye(3,3,CV_64FC1);

	//image->copyTo(src);

	float markerSize = 0.17;
	// float markerSize = 0.1;

	vector<int> markerIds;
	vector<vector<Point2f> > corners, rejectedCandidates;
	vector<Vec3d> rvecs, tvecs;


	// aruco::Dictionary dictionary=aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
	//aruco::Dictionary dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

   	Mat cameraMatrix=(Mat_<double>(3,3)<<633.06058204,0,330.28981083,0,631.01252673,226.42308878,0,0,1);
	Mat distCoeffs=(Mat_<double>(1,5)<< 0.0503468649,-0.0438421987, -0.000252895273 , 0.00191361583,-0.490955908);

	pose_t cam_pos;
	cam_pos.valid = false;
	cam_pos.x = 0;
	cam_pos.y = 0;
	cam_pos.z = 0;
	cam_pos.th = 0;

	if( image->empty() ) {
		cout << "ERROR NO PICTURE!!!\n";
		cam_pos.valid = 0;
		return cam_pos;
	}
  
    //aruco::detectMarkers(src, dictionary, corners, markerIds); 
    aruco::detectMarkers(*image, dictionary, corners, markerIds); 

    if( draw_markers ) aruco::drawDetectedMarkers( *image, corners, markerIds);
    
    if (markerIds.size() > 0){

        int known_markers = 0;
        for(unsigned int i = 0; i < markerIds.size(); i++)
        {
     		if( markerIds[i] != 0 ) continue;
            

            cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs); 
        
            Rodrigues(rvecs[i], R33);
            Vec3f angles = rotationMatrixToEulerAngles(R33);
            
            cam_pos.x = tvecs[i][2];
            cam_pos.y = -tvecs[i][0];
            cam_pos.z = -tvecs[i][1];
            cam_pos.th = atan2(cam_pos.y, cam_pos.x);
            cam_pos.valid = 1;
//              if( draw_marker ) aruco::drawAxis( *image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
                
            }
        }

	return cam_pos;
	
}

