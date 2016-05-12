#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

#define _FX 1.0
#define _FY 1.0
#define _CX 1.0
#define _CY 1.0

using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "world_coordinates_estimation");

	const double cam_mat[3][3] = {
		{_FX, 0, _CX},
		{0, _FY, _CY},
		{0,   0,   1}
	};

	// cam_mat^-1 * pixelcoords = realworldcoords

	//TODO: CREATE MY OWN MESSAGES FOR THE KEYPOINTS

	return 0;
}