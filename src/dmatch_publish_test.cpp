#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

#include "tfg/dmatch.h"
#include "tfg/dmatchArray.h"

using namespace cv;

void matches_callback(const tfg::dmatchArray::ConstPtr& msg)
{
	for(int i=0; i<msg->dmatches.size(); ++i)
	{
		const tfg::dmatch &data = msg->dmatches[i];
		ROS_INFO_STREAM("queryIdx: " << data.queryIdx << "trainIdx: " << data.trainIdx <<
			"imgIdx: " << data.imgIdx << "distance: " << data.distance);
	}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmatch_publish_test");
    ros::NodeHandle n;

    ros::Subscriber matches_sub = n.subscribe("dmatches", 5, matches_callback);

    ros::spin();
    return 0;
}