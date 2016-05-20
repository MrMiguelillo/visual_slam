#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "tfg/dmatch.h"
#include "tfg/dmatchArray.h"
#include "tfg/keypoint.h"
#include "tfg/keypointArray.h"
#include "tfg/pointsMatched.h"
#include "tfg/pointsMatchedArray.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

using namespace cv;

Mat left_img;
Mat right_img;

void left_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	left_img = cv_bridge::toCvCopy(msg, "mono8")->image;
}

void right_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	right_img = cv_bridge::toCvCopy(msg, "mono8")->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_feature_matching");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber left_sub = it.subscribe("/left/image_raw", 1, left_img_callback);
    image_transport::Subscriber right_sub = it.subscribe("/right/image_raw", 1, right_img_callback);

    //ros::Publisher matches_pub = n.advertise<tfg::dmatchArray>("dmatches", 5);
    //ros::Publisher keypoints_left_pub = n.advertise<tfg::keypointArray>("keypoints/left", 5);
    //ros::Publisher keypoints_right_pub = n.advertise<tfg::keypointArray>("keypoints/right", 5);
    ros::Publisher matches_pub = n.advertise<tfg::pointsMatchedArray>("matches", 5);

    ros::Rate loop_rate(2);
    ros::Duration(1).sleep();
    ros::spinOnce();

    ros::Time::init();
    while(!ros::Time::isValid());
    ros::Time begin  = ros::Time::now();
    ros::Time end;

    bool already_writen = false;

    while (ros::ok())
	{
		std::vector<cv::KeyPoint> left_keypoints;
    	std::vector<cv::KeyPoint> right_keypoints;
    	Mat left_mask = cv::Mat::ones(left_img.size(), CV_8U);
    	Mat right_mask = cv::Mat::ones(right_img.size(), CV_8U);
    	Mat left_descriptors;
    	Mat right_descriptors;

    	SURF detector(400);
    	detector(left_img, left_mask, left_keypoints, left_descriptors, false);
    	detector(right_img, right_mask, right_keypoints, right_descriptors, false);

    	if(left_descriptors.type() != CV_32F){
        	left_descriptors.convertTo(left_descriptors, CV_32F);
    	}

    	if(right_descriptors.type() != CV_32F){
        	right_descriptors.convertTo(right_descriptors, CV_32F);
    	}

		cv::FlannBasedMatcher matcher;
	    std::vector<std::vector<cv::DMatch> > matches;
	    matcher.knnMatch( left_descriptors, right_descriptors, matches, 2 );

	    double thresholdDist2 = 0.0625 * double(left_img.size().height*left_img.size().height + 
							left_img.size().width*left_img.size().width);

        tfg::pointsMatched match_data;
        tfg::pointsMatchedArray match_msg;
	    //std::vector< cv::DMatch > good_matches2;
		//good_matches2.reserve(matches.size());
        match_msg.matches.reserve(matches.size());
		for (size_t i = 0; i < matches.size(); ++i)
		{ 
		    for (int j = 0; j < matches[i].size(); j++)
		    {
	        	Point2f from = left_keypoints[matches[i][j].queryIdx].pt;
	        	Point2f to = right_keypoints[matches[i][j].trainIdx].pt;

				double dist2 = (from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y);
		
	        	if (dist2 < thresholdDist2 && abs(from.y-to.y)<3)
	        	{
		            //good_matches2.push_back(matches[i][j]);
	            	j = matches[i].size();
                    match_data.fromx = from.x;
                    match_data.fromy = from.y;
                    match_data.tox = to.x;
                    match_data.toy = to.y;
                    //match_data.size = left_keypoints[matches[i][j].queryIdx].size;
                    match_msg.matches.push_back(match_data);
	        	}
	    	}
		}
        matches_pub.publish(match_msg);
		
        /*
		tfg::dmatch dmatch_data;
        tfg::dmatchArray dmatch_msg;
        for(int i=0; i<good_matches2.size(); i++)
        {
            dmatch_data.queryIdx = good_matches2[i].queryIdx;
            dmatch_data.trainIdx = good_matches2[i].trainIdx;
            dmatch_data.imgIdx = good_matches2[i].imgIdx;
            dmatch_data.distance = good_matches2[i].distance;
            dmatch_msg.dmatches.push_back(dmatch_data);
        }

        tfg::keypoint keypoint_data;
        tfg::keypointArray keypoint_msg;
        for(int i=0; i<good_matches2.size(); i++)
        {
            keypoint_data.x = left_keypoints[i].pt.x;
            keypoint_data.y = left_keypoints[i].pt.y;
            keypoint_data.size = left_keypoints[i].size;
            keypoint_msg.keypoints.push_back(keypoint_data);
        }
        keypoints_left_pub.publish(keypoint_msg);

        for(int i=0; i<good_matches2.size(); i++)
        {
            keypoint_data.x = right_keypoints[i].pt.x;
            keypoint_data.y = right_keypoints[i].pt.y;
            keypoint_data.size = right_keypoints[i].size;
            keypoint_msg.keypoints.push_back(keypoint_data);
        }
        keypoints_right_pub.publish(keypoint_msg);

        if(!already_writen)
        {
            cv::Mat img_matches;
            vector< cv::DMatch > some_good_matches;
            some_good_matches.reserve(5);
            cv::namedWindow("Good Matches");

            for(int i=0, j=0; i<good_matches2.size(); i+=5, j++){

                if(5*(j+1) <= good_matches2.size()){
                    some_good_matches.clear();
                    for(int a=0; a<5; a++){
                        some_good_matches.push_back(good_matches2[i+a]);
                    }
                }
                else{
                    some_good_matches.clear();
                    for(int a=0; a<(good_matches2.size() - j*5); a++){
                        some_good_matches.push_back(good_matches2[i+a]);
                    }
                }
                //some_good_matches.clear();
                //some_good_matches.push_back(good_matches2[i]);
                cv::drawMatches( left_img, left_keypoints, right_img, right_keypoints,
                           some_good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                cv::imshow( "Good Matches", img_matches );
                cv::waitKey();
            }

            cv::destroyWindow("Good Matches");
            already_writen = true;
        }*/

    	ros::spinOnce();
    	loop_rate.sleep();
    	end = ros::Time::now();
    	std::cout << "Time spent on iteration: " << end-begin << std::endl;
    	begin = ros::Time::now();
    }

    return 0;
}