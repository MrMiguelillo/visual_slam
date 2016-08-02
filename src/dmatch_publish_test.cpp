#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>
#include <algorithm> //for max function

#include "tfg/dmatch.h"
#include "tfg/dmatchArray.h"
#include "tfg/keypoint.h"
#include "tfg/keypointArray.h"
#include "tfg/pointsMatched.h"
#include "tfg/pointsMatchedArray.h"

cv::Mat left_img, right_img ,resImg;
void drawMatchedPoints(const std::vector<tfg::pointsMatched>& points);
void initializeResultsWindow();
//std::vector<cv::KeyPoint> left_keypoints;
//std::vector<cv::KeyPoint> right_keypoints;
/*
void left_keypoints_callback(const tfg::keypointArray::ConstPtr& msg)
{
	left_keypoints.clear();
	left_keypoints.resize(msg->keypoints.size());
	for(int i=0; i<msg->keypoints.size(); i++)
	{
		left_keypoints[i] = cv::KeyPoint(msg->keypoints[i].x, msg->keypoints[i].y, msg->keypoints[i].size);
	}

	std::cout << "left_keypoints recieved" << std::endl;
}


void right_keypoints_callback(const tfg::keypointArray::ConstPtr& msg)
{
	right_keypoints.clear();
	right_keypoints.resize(msg->keypoints.size());
	for(int i=0; i<msg->keypoints.size(); i++)
	{
		right_keypoints[i] = cv::KeyPoint(msg->keypoints[i].x, msg->keypoints[i].y, msg->keypoints[i].size);
	}

	std::cout << "right_keypoints recieved" << std::endl;
}
*/
//void matches_callback(const tfg::dmatchArray::ConstPtr& msg)
void matches_callback(const tfg::pointsMatchedArray::ConstPtr& msg)
{
	std::vector<tfg::pointsMatched> data;
	data.reserve(msg->matches.size());
	static bool already_writen = false;

	for(int i=0; i<msg->matches.size(); ++i)
	{
		data.push_back(msg->matches[i]);
	}
	/*
	cv::Mat img_matches;
	std::vector< cv::DMatch > some_good_matches;
	for(int i=0; i<10; ++i)
	{
		some_good_matches.push_back(cv::DMatch(data[i].queryIdx, data[i].trainIdx, data[i].imgIdx, data[i].distance));
	}

	if(left_keypoints.size()>0 && right_keypoints.size()>0 && !already_writen)
	{
		std::cout << some_good_matches.size() << std::endl;
		cv::drawMatches( left_img, left_keypoints, right_img, right_keypoints,
	               some_good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		imwrite( "/home/mike/catkin_ws/src/tfg/images/result_matching_img.jpg", img_matches );
		already_writen = true;
	}
	*/

	drawMatchedPoints(data);
	std::cout << "matches data recieved" << std::endl;
}

void drawMatchedPoints(const std::vector<tfg::pointsMatched>& points)
{
	for(int i=0; i<points.size(); i++)
	{
		std::cout << points[i].fromx << points[i].toy << std::endl;
		cv::circle(resImg, cv::Point(points[i].fromx, points[i].fromy), 3, cv::Scalar(255,0,255));
		cv::circle(resImg, cv::Point(points[i].tox + left_img.cols, points[i].toy), 3, cv::Scalar(255,0,255));

		cv::line(resImg, cv::Point(points[i].fromx, points[i].fromy), cv::Point(points[i].tox + left_img.cols, points[i].toy), 
			cv::Scalar(255,0,255));
	}
}

void initializeResultsWindow()
{
	cv::Size left_sz = left_img.size();
	cv::Size right_sz = right_img.size();

	cv::Mat left_subimage = resImg( cv::Rect(0, 0, left_img.cols, left_img.rows) );
	left_img.copyTo( left_subimage );

	cv::Mat right_subimage = resImg( cv::Rect(left_img.cols, 0, right_img.cols, right_img.rows) );
	right_img.copyTo( right_subimage );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmatch_publish_test");
    ros::NodeHandle n;

    ros::Subscriber matches_sub = n.subscribe("matches", 5, matches_callback);
    //ros::Subscriber matches_sub = n.subscribe("dmatches", 5, matches_callback);
    //ros::Subscriber left_keypoints_sub = n.subscribe("keypoints/left", 5, left_keypoints_callback);
    //ros::Subscriber right_keypoints_sub = n.subscribe("keypoints/right", 5, right_keypoints_callback);

    left_img = cv::imread("/home/mike/catkin_ws/src/tfg/images/left.jpg", CV_LOAD_IMAGE_COLOR);
    right_img = cv::imread("/home/mike/catkin_ws/src/tfg/images/right.jpg", CV_LOAD_IMAGE_COLOR);

    //left_keypoints.clear();
    //right_keypoints.clear();

    cv::Mat auxImg(std::max(left_img.rows, right_img.rows), left_img.cols + right_img.cols, left_img.type());
    resImg = auxImg;

    initializeResultsWindow();

    cv::namedWindow("test window");
    imshow("test window", resImg);
    cv::waitKey();
    //drawMatchedPoints();
    //ros::Rate loop_rate(2);

    while(ros::ok())
    {
    	ros::spinOnce();
    	imshow("test window", resImg);
    	cv::waitKey();
    	//loop_rate.sleep();
    }
    cv::destroyWindow("test window");
    return 0;
}

// TODO:
//		1.modificar initializeResultsWindow para que tome por parámetro una imagen y hacer que resImg NO SEA GLOBAL
// 		2.ver si left_img y right_img pueden dejar de ser globales
	