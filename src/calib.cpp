//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdio.h>
//Store all constants for image encodings in the enc namespace to be used later.

 using namespace cv;
using namespace std;

 
 
 
int main(int argc, char **argv)
{

   ros::init(argc, argv, "calib");
    
        ros::NodeHandle nh;
        
        
      int numBoards = 3;
    int board_w = 3;
    int board_h = 7;

    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
    vector<Point2f> corners;

    vector<Point3f> obj;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

    Mat img, gray;
    VideoCapture cap = VideoCapture(0);

    int success = 0;
    int k = 0;
    bool found = false;
    
    
    
 while (success < numBoards)
    {
        cap >> img;
        cvtColor(img, gray, CV_BGR2GRAY);
        found = findChessboardCorners(gray, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found)
        {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray, board_sz, corners, found);
        }

        imshow("image", img);
        imshow("corners", gray);
        
        k = waitKey(1);
        if (found)
        {
            k = waitKey(0);
            ROS_INFO("%d",k);
        }
        if (k == 27)
        {
            break;
        }
        if (k == 1048608 && found !=0)
        {
            image_points.push_back(corners);
            object_points.push_back(obj);
            printf ("Corners stored\n");
            success++;

            if (success >= numBoards)
            {
                break;
            }
        }

    }
    destroyAllWindows();
    printf("Starting calibration\n");
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distcoeffs;
    vector<Mat> rvecs, tvecs;

    intrinsic.at<float>(0, 0) = 1;
    intrinsic.at<float>(1, 1) = 1;
    
    calibrateCamera(object_points, image_points, img.size(), intrinsic, distcoeffs, rvecs, tvecs);

    FileStorage fs1("mycalib.yml", FileStorage::WRITE);
    fs1 << "CM1" << intrinsic;
    fs1 << "D1" << distcoeffs;

    printf("calibration done\n");

    Mat imgU;
    while(1)
    {
        cap >> img;
        undistort(img, imgU, intrinsic, distcoeffs);

        imshow("image", img);
        imshow("undistort", imgU);

        k = waitKey(5);
        if (k == 27)
        {
            break;
        }
    }
    cap.release();
    return(0);
}
