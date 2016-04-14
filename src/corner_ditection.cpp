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
//Store all constants for image encodings in the enc namespace to be used later.

 using namespace cv;
using namespace std;

 int blockSize = 2;
  int apertureSize = 3;
 double k = 0.04;
int max_thresh = 255;
 int thresh = 200;
 Mat imgTmp;
Mat src, src_gray;
 Mat dst, dst_norm, dst_norm_scaled;
 
 
int main(int argc, char **argv)
{

   ros::init(argc, argv, "corner_ditection");
    
        ros::NodeHandle nh;

  VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

namedWindow("Control",CV_WINDOW_AUTOSIZE);
cvCreateTrackbar("Threshold: ", "Control", &thresh,255);


   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
 namedWindow("Corners detected",CV_WINDOW_AUTOSIZE);

 cap.read(imgTmp);
 dst = Mat::zeros( imgTmp.size(), CV_32FC1 ); 

    while (ros::ok())
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        
        
            src=frame;
cvtColor( src, src_gray, CV_BGR2GRAY );
      

 cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  imgTmp=Mat::zeros( imgTmp.size(), CV_32FC1 );
int n=0;
  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh && n< 10)
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
			circle( src_gray, Point( i, j ), 5,  Scalar(200), 2, 8, 0 );
               n++;
              }
          }
     }
  /// Showing the result

  imshow("Corners detected", dst_norm_scaled );
       imshow("MyVideo", src_gray); //show the frame in "MyVideo" window 

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
    return 0;
}
