#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_matching");

    const cv::Mat left_img = cv::imread("/home/mike/catkin_ws/src/tfg/images/left.jpg", 0);
    const cv::Mat right_img = cv::imread("/home/mike/catkin_ws/src/tfg/images/right.jpg", 0);

    if( !left_img.data || !right_img.data ){
        printf(" --(!) Error reading images \n"); return -1;
    }


//----STEP 1: DETECT FEATURES
    cv::SIFT detector(0, 4, 0.03);
    cv::Mat left_mask = cv::Mat::ones(left_img.size(), CV_8U);
    cv::Mat right_mask = cv::Mat::ones(right_img.size(), CV_8U);
    std::vector<cv::KeyPoint> left_keypoints;
    std::vector<cv::KeyPoint> right_keypoints;
    cv::Mat left_descriptors;
    cv::Mat right_descriptors;

    detector(left_img, left_mask, left_keypoints, left_descriptors);
    detector(right_img, right_mask, right_keypoints, right_descriptors);

    cv::Mat left_output, right_output;
    cv::drawKeypoints(left_img, left_keypoints, left_output);
    //cv::drawKeypoints(right_img, right_keypoints, right_output);

    cv::imshow("left", left_img);
    cv::imshow("left keypoints", left_output);
    cv::waitKey();


//----STEP 2: MATCH DESCRIPTORS USING FLANN
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch( left_descriptors, right_descriptors, matches, 2 );

    double min_dist = 1000;
    double max_dist = 0;
    for( int i = 0; i < left_descriptors.rows; i++ ){
        double dist = matches[i][0].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }


    std::vector< cv::DMatch > good_matches;
    for (int i = 0; i < matches.size(); ++i){
        const float ratio = 0.8; // As in Lowe's paper; can be tuned
        if (matches[i][0].distance < ratio * matches[i][1].distance && matches[i][0].distance <= std::max(2*min_dist, 0.02)){
            good_matches.push_back(matches[i][0]);
        }
    }

    cv::Mat img_matches;
    cv::drawMatches( left_img, left_keypoints, right_img, right_keypoints,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    cv::imshow( "Good Matches", img_matches );

    
    for( int i = 0; i < (int)good_matches.size(); i++ ){
        printf( "-- Good Match [%d] Distance %f  \n", i, good_matches[i].distance );
    }

    cv::waitKey();
    
    return 0;
}