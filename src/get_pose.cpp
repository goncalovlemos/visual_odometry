#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sstream>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ctype.h>

//---------------------------------------------

#define MAX_FEATURE_COUNT 200
#define PRINT_ALL 1
#define PRINT_FEATURES_LOCATION 1
#define SHOW_VIDEO_FEED 0
#define SHOW_FLOW 1

//---------------------------------------------

using namespace cv;
using namespace std;

//--------------------------------------------- Global variables

int needtoInit = 1;
Mat mask;
cv::Mat image, previmage;
vector<Point2f> corners, prevcorners;
int ofcnt = 0;

void calcCallback(const std_msgs::Float32MultiArray dat)
{

	int W = 70;
	//int W = dat.layout.dim[1]; //change for height
	printf("|-------------------------------------------------------|\n\n");
	int cnt = 1;
	for (int i=0; i<W; i = i + 2){ 
		float cx = dat.data[i];
		float cy = dat.data[i+1];
		printf("Feature %d: ( %f , %f ) \n",cnt,cx,cy);
		cnt++;
		//dat.data.push_back(prevcorners[i].x);
		//dat.data.push_back(prevcorners[j].y);
	}



}

int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "get_pose");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  cv::startWindowThread();
  ros::Subscriber sub = nh.subscribe("uwv_2d_data", 1, calcCallback);
  printf("\n\nWaiting for point 2d vector topic...\n");
  ros::spin();
  //cv::destroyWindow("view");
}


