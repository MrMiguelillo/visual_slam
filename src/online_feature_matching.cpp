#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

using namespace cv;

Mat left_img, right_img;

void left_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	left_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

void right_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	right_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_feature_matching");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber left_sub = it.subscribe("left/image_raw", 1, left_img_callback);
    image_transport::Subscriber right_sub = it.subscribe("right/image_raw", 1, right_img_callback);

    ros::Rate loop_rate(3);

    while (ros::ok())
	{
		
		
		std_msgs::String msg;
		std::stringstream ss;
		ss <<  << ;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

    	chatter_pub.publish(msg);

    	ros::spinOnce();
    	loop_rate.sleep();
    }

    return 0;
}