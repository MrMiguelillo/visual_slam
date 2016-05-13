#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_feeder");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    image_transport::Publisher left_pub = it.advertise("/left/image_raw", 1);
    image_transport::Publisher right_pub = it.advertise("/right/image_raw", 1);

    const Mat left_img = imread("/home/mike/catkin_ws/src/tfg/images/left.jpg", CV_LOAD_IMAGE_COLOR);
    const Mat right_img = imread("/home/mike/catkin_ws/src/tfg/images/right.jpg", CV_LOAD_IMAGE_COLOR);

    cv_bridge::CvImagePtr left_ros_img(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr right_ros_img(new cv_bridge::CvImage);

    left_ros_img->image = left_img;
    left_ros_img->encoding = "bgr8";
    right_ros_img->image = right_img;
    right_ros_img->encoding = "bgr8";

    ros::Rate loop_rate(2);

    while(ros::ok()){
    	//publish images at 2hz
    	left_pub.publish(left_ros_img->toImageMsg());
    	right_pub.publish(right_ros_img->toImageMsg());
    	loop_rate.sleep();
    }

    return 0;
}