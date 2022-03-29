#include "BallLocalizer.hpp"

// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"

#include <math.h>
#include "rotations.hpp"
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;

float point_distance2( Point2f a, Point2f b){
    float dist = sqrt( pow( (a.x - b.x), 2) + pow(a.y - b.y, 2) );
    return dist;
}

void BallLocalizer::init( float ball_r ){
    
    ball_points[0] = Point3f(0,0,ball_r);
    ball_points[1] = Point3f(0,-ball_r,0);
    ball_points[2] = Point3f(0,0,-ball_r);
    ball_points[3] = Point3f(0,ball_r,0);
    ball_points[4] = Point3f(0,0,0);

}

pose_t BallLocalizer::update2(cv::Mat * image, float roll, float pitch){
    
    pose_t pose_est;
    pose_est.x = 0;
    pose_est.y = 0;
    pose_est.z = 0;
    pose_est.th = 0;
    pose_est.valid = 0;
    
    Mat hsv;
    
    //cvtColor( *image, hsv, CV_RGB2HSV);
    cvtColor( *image, hsv, COLOR_RGB2HSV);

// 	inRange( hsv, Scalar(100,25,25), Scalar(140,255,255), hsv );
    inRange( hsv, Scalar(0,70,70), Scalar(12 ,255,255), hsv );

    
    
	blur(hsv, hsv, Size(5,5));

	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(5,5), Size(-1,-1));

	erode(hsv, hsv, element);
	dilate(hsv, hsv, element);

    imshow( "hsv", hsv );
    
// 	blur(hsv, hsv, Size(5,5));

  vector<Vec3f> circles;

  if( lost == lost_limit ){

  	/// Apply the Hough Transform to find the circles
	  HoughCircles( hsv, circles, HOUGH_GRADIENT, 1, 800, 150, 8, 1, 100 );

	  if( circles.size() > 0 ){
	  	lost = 0;
        array_counter = 0;
        array_place = 0;
	  }

  } 
  else{
  	int crop_extra = 50;

  	int top = (int) old_position[1] - crop_extra;
  	int bot = (int) old_position[1] + crop_extra;
  	int left = (int) old_position[0] - crop_extra;
  	int right = (int) old_position[0] + crop_extra;

  	if( top < 0 ) top = 0;
  	if( bot > 479 ) bot = 479;
  	if( left < 0 ) left = 0;
  	if( right > 639 ) right = 639;

  	Mat subHsv = hsv(Range(top,bot), Range(left, right));

//     imshow("subframe", subHsv);
//     imshow("hsv", hsv);
    
  	//HoughCircles( subHsv, circles, CV_HOUGH_GRADIENT, 1, 800, 150, 12, 1, 100 );
    HoughCircles( subHsv, circles, HOUGH_GRADIENT, 1, 800, 150, 12, 1, 100 );

  	if( circles.size() > 0){

  		if (circles[0][2] > 0.1){
  			circles[0][1] += top;
  			circles[0][0] += left;

  			lost = 0;
  		}
  		else {
  			lost++;
  		}
  	}
  	else{
  		lost++;
  	}


  }
  if( lost == 0 ) {

  		old_position[0] = circles[0][0];
  		old_position[1] = circles[0][1];

        float x = circles[0][0];
        float y = circles[0][1];
        float r = circles[0][2];
        
        image_points[0] = Point2f(x, y-r);
        image_points[1] = Point2f(x+r, y);
        image_points[2] = Point2f(x, y+r);
        image_points[3] = Point2f(x-r, y);
        image_points[4] = Point2f(x, y);
        
        Vec3d rvec, tvec;
        
        solvePnP( ball_points, image_points, cameraMatrix, distCoeffs, rvec, tvec);
        
//         Mat R33 = Rodrigues(rvec);
        
        avg_array[0][array_place] = tvec[0];
        avg_array[1][array_place] = tvec[1];
        avg_array[2][array_place] = tvec[2];
        avg_array[3][array_place] = 0;
        
        array_place++;
        if( array_place == 5) array_place = 0;
        
        if( array_counter < 5 ) array_counter++;
            
        
        float avg_x = 0;
        float avg_y = 0;
        float avg_z = 0;
        float avg_th = 0;
        
        for( int i = 0; i <= array_counter; i++ ) {
            
            avg_x += avg_array[0][i];
            avg_y += avg_array[1][i];
            avg_z += avg_array[2][i];
            avg_th += avg_array[3][i];
            
        }
        
        avg_x /= array_counter;
        avg_y /= array_counter;
        avg_z /= array_counter;
        avg_th /= array_counter;
        
        pose_est.x = -avg_z;
        pose_est.y = avg_x;
        pose_est.z = avg_y;
        pose_est.th = avg_th;
        pose_est.valid = 1;
        
  	}
    
    if( circles.size() > 0 ) {
        Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
        int radius = cvRound(circles[0][2]);
        circle(*image, center, 3, Scalar(0,255,255), -1, 8, 0);
        circle( *image, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    
    return pose_est;

    
}

pose_t BallLocalizer::update(cv::Mat * image, float roll, float pitch)
{

	//Mat src;
	Mat R33 = Mat::eye(3,3,CV_64FC1);

	//image->copyTo(src);

	float markerSize;

	vector<int> markerIds;
	vector<vector<Point2f> > corners, rejectedCandidates;
	vector<Vec3d> rvecs, tvecs;
    
   	Mat cameraMatrix=(Mat_<double>(3,3)<<633.06058204,0,330.28981083,0,631.01252673,226.42308878,0,0,1);
	Mat distCoeffs=(Mat_<double>(1,5)<< 0.0503468649,-0.0438421987, -0.000252895273 , 0.00191361583,-0.490955908);

	pose_t cam_pos;
	cam_pos.valid = false;
	cam_pos.x = 0;
	cam_pos.y = 0;
	cam_pos.z = 0;
	cam_pos.th = 0;

	float x_avg = 0;
	float y_avg = 0;
	float z_avg = 0;
	float yaw_avg = 0;

	//cap->grab();
	//cap->retrieve(src);

	//if( !src.data ) {
	if( image->empty() ) {
		cout << "ERROR NO PICTURE!!!\n";
		cam_pos.valid = 0;
		return cam_pos;
	}

    
    Mat gray;

    Mat hsv, low_mask, high_mask, mask;
    Mat rgb, masked_hsv;
    cvtColor( *image, hsv, COLOR_RGB2HSV );
    
    inRange( hsv, Scalar(0,25,25), Scalar(14,255,255), low_mask );
    //inRange( hsv, Scalar(145,25,25), Scalar(180,255,255), high_mask );
    
    //bitwise_or( low_mask, high_mask, mask);
    
    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(5,5), Size(-1,-1));

    
    mask = low_mask;
        erode(mask, mask, element);
        dilate(mask, mask, element);

        imshow( "hsv", mask );

    
//     bitwise_and(hsv, hsv, masked_hsv, mask);
            
//     cvtColor( masked_hsv, rgb, COLOR_HSV2RGB );
//     cvtColor( rgb, gray, COLOR_RGB2GRAY );
    
//     blur( gray, gray, Size(5,5));
//     blur( mask, mask, Size(5,5));
    
//     adaptiveThreshold( gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 9, 0);
    
//         Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3,3), Point(-1,-1));
    
//         dilate( gray, gray, element);
    
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    
            float area, infill, aspect;
        float l1, l2, l3, l4, min_l, max_l, cx, cy;
        RotatedRect box;
    //         vector<Point> bpoint;
        
        Point2f bpoint[4];
        
        int best_contour = -1;
        float best_aspect = 0;
    
      if( lost == lost_limit ){

            /// Apply the Hough Transform to find the circles
        // 	  HoughCircles( hsv, circles, CV_HOUGH_GRADIENT, 1, 800, 150, 8, 1, 100 );
            findContours( mask, contours, hierachy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));

            
            for( int i = 0; i < contours.size(); i++) {
                area = contourArea( contours[i] );
                
                if( area > 200 && area < (640*480*0.4) ){
                    
                    box = minAreaRect(contours[i]);
                    
                    box.points(bpoint);
                    
                    l1 = point_distance2(bpoint[0], bpoint[1]);
                    l2 = point_distance2(bpoint[1], bpoint[2]);
                    l3 = point_distance2(bpoint[2], bpoint[3]);
                    l4 = point_distance2(bpoint[3], bpoint[0]);
                    
                    min_l = l1;
                    if(l2 < min_l) min_l = l2;
                    else if( l3 < min_l) min_l = l3;
                    else if( l4 < min_l) min_l = l4;
                    max_l = l1;
                    if(l2 > max_l) max_l = l2;
                    else if( l3 > max_l) max_l = l3;
                    else if( l4 > max_l) max_l = l4;
                    
                    infill = contourArea(contours[i]) / (min_l*max_l);
                    
                    aspect = min_l / max_l;
                    
        //                 std::cout << "l1 = " << l1 << std::endl;
        //                 std::cout << "l2 = " << l2 << std::endl;
        //                 std::cout << "l3 = " << l3 << std::endl;
        //                 std::cout << "l4 = " << l4 << std::endl;
        //                 std::cout << "mn = " << min_l << std::endl;
        //                 std::cout << "mx = " << max_l << std::endl;
        //                 std::cout << "if = " << infill << std::endl;
        //                 std::cout << "ar = " << aspect << std::endl;
                    
//                     drawContours( *image, contours, i, Scalar(0,0,255), 2, 8, hierachy, 0, Point() );
                    // Box must be large enough
                        // Box must have correct aspect ratio
                        if( (aspect > 0.5 ) && (aspect < 1.5) ){
        //                         std::cout << aspect << std::endl;
                            // Box must be at least 35% solid
                            if( infill > 0.30 ){
        //                             std::cout << infill << std::endl;
                                // Box needs to be better then last box
                                if(  abs(aspect - 0.5) < abs(best_aspect - 0.5) ){
                                    best_aspect = aspect;
                                    best_contour = i;
                                }
                            }
                    }
                        
                }
                
            }
    
	  if( best_contour != -1 ){
	  	lost = 0;
        array_counter = 0;
        array_place = 0;
        
        Moments M = moments(contours[best_contour]);
        
        cx = M.m10 / M.m00;
        cy = M.m01 / M.m00;
        
//                         std::cout << cx << "  cx1\n";
//                 std::cout << cy << "  cy1\n";
        
	  }

    } 
    else{
        
          	int crop_extra = 100;

  	int top = (int) old_position[1] - crop_extra;
  	int bot = (int) old_position[1] + crop_extra;
  	int left = (int) old_position[0] - crop_extra;
  	int right = (int) old_position[0] + crop_extra;

//     std::cout << old_position[1] << " " << old_position[0] << "\n";
//     std::cout << top << " " << bot << " " << left << " " << right << "\n";
//     
  	if( top < 0 ) top = 0;
  	if( bot > 479 ) bot = 479;
  	if( left < 0 ) left = 0;
  	if( right > 639 ) right = 639;

  	Mat subMask = mask(Range(top,bot), Range(left, right));
        
        findContours( subMask, contours, hierachy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
        

        
        for( int i = 0; i < contours.size(); i++) {
            area = contourArea( contours[i] );
            
            if( area > 100 && area < (640*480*0.4) ){
                
                box = minAreaRect(contours[i]);
                
                box.points(bpoint);
                
                l1 = point_distance2(bpoint[0], bpoint[1]);
                l2 = point_distance2(bpoint[1], bpoint[2]);
                l3 = point_distance2(bpoint[2], bpoint[3]);
                l4 = point_distance2(bpoint[3], bpoint[0]);
                
                min_l = l1;
                if(l2 < min_l) min_l = l2;
                else if( l3 < min_l) min_l = l3;
                else if( l4 < min_l) min_l = l4;
                max_l = l1;
                if(l2 > max_l) max_l = l2;
                else if( l3 > max_l) max_l = l3;
                else if( l4 > max_l) max_l = l4;
                
                infill = contourArea(contours[i]) / (min_l*max_l);
                
                aspect = min_l / max_l;
                
    //                 std::cout << "l1 = " << l1 << std::endl;
    //                 std::cout << "l2 = " << l2 << std::endl;
    //                 std::cout << "l3 = " << l3 << std::endl;
    //                 std::cout << "l4 = " << l4 << std::endl;
    //                 std::cout << "mn = " << min_l << std::endl;
    //                 std::cout << "mx = " << max_l << std::endl;
    //                 std::cout << "if = " << infill << std::endl;
    //                 std::cout << "ar = " << aspect << std::endl;
                
//                 drawContours( subMask, contours, i, Scalar(0,0,255), 2, 8, hierachy, 0, Point() );
//                 imshow("submac",subMask);
                // Box must be large enough
                    // Box must have correct aspect ratio
                    if( (aspect > 0.5 ) && (aspect < 1.5) ){
    //                         std::cout << aspect << std::endl;
                        // Box must be at least 35% solid
                        if( infill > 0.30 ){
    //                             std::cout << infill << std::endl;
                            // Box needs to be better then last box
                            if(  abs(aspect - 0.5) < abs(best_aspect - 0.5) ){
                                best_aspect = aspect;
                                best_contour = i;
                            }
                        }
                }
                    
            }
            
        }
            if( best_contour != -1 ){
                            Moments M = moments(contours[best_contour]);
        
                cx = M.m10 / M.m00 + (float)left;
                cy = M.m01 / M.m00 + (float)top;
        
//                 std::cout << cx << "  cx\n";
//                 std::cout << cy << "  cy\n";

                    lost = 0;
                }
                else {
                    lost++;
                }
            
                
    }
    
    if( lost == 0 ) {
            std::cout << "Found a good box\n";

//         std::cout << cy << "  " << cx << "\n";
        old_position[1] = cy;
  		old_position[0] = cx;

        float x = -0.14/(max_l/632.0);
        
//         if( camera->show_image ){
            //drawContours( *image, contours, best_contour, Scalar(0,255,255), 2, 8, hierachy, 0, Point() );
            circle( *image,Point(cvRound(cx),cvRound(cy)),1,Scalar(0,0,255),5,8,0);
//         }
        
//         x = (x - 320) * camera->get_last_height()/630.0 + camera->get_last_height()*tan(roll);
//         y = (y - 240) * camera->get_last_height()/630.0 + camera->get_last_height()*tan(pitch);
        
//             std::cout << y << "   ||   " << x << std::endl;
        
        Mat pose = (Mat_<double>(3,1) <<
            (cx - 320),       (cy - 240),              0.00
        );
        
        Vec3f angles = Vec3f(0,0,roll);
        
//             std::cout << "nadsnsles\n";
        Mat RotYaw = eulerAnglesToRotationMatrix( angles );
        
        pose = RotYaw * pose;
        
        float y = (pose.at<double>(0))*x/632;
        
        avg_array[0][array_place] = x;
        avg_array[1][array_place] = y;
        avg_array[2][array_place] = 0;
        avg_array[3][array_place] = 0;
        
        array_place++;
        if( array_place == 5) array_place = 0;
        
        if( array_counter < 5 ) array_counter++;
            
        
        float avg_x = 0;
        float avg_y = 0;
        float avg_z = 0;
        float avg_th = 0;
        
        for( int i = 0; i <= array_counter; i++ ) {
            
            avg_x += avg_array[0][i];
            avg_y += avg_array[1][i];
            avg_z += avg_array[2][i];
            avg_th += avg_array[3][i];
            
        }
        
        avg_x /= array_counter;
        avg_y /= array_counter;
        avg_z /= array_counter;
        avg_th /= array_counter;
        
        cam_pos.x = avg_x;
        cam_pos.y = avg_y;
//         cam_pos.z = camera->get_last_height();
//         cam_pos.th = camera->get_last_heading();
        cam_pos.valid = true;
    }

	return cam_pos;
	
}
