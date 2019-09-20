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

	//int H = dat.layout.dim[0].size;
	int H = dat.data[0];
	printf("\n\n[%i features] \n|-------------------------------------------------------|\n", H);
	int cnt = 0;

	for (int i=1; i < (H*2); i = i + 2){
		cnt++;

		if(dat.data[i] > 0 && dat.data[i+1] > 0){
		float cx = dat.data[i];
		float cy = dat.data[i+1];
		printf("Feature %d: ( %f , %f ) \n",cnt,cx,cy);
		}
		//guardar dados num vetor unico
	}
	//dat.data.clear();


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


