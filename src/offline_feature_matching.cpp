#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>

using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_feature_matching");

    const Mat left_img = imread("/home/mike/catkin_ws/src/tfg/images/left.jpg", 0);
    const Mat right_img = imread("/home/mike/catkin_ws/src/tfg/images/right.jpg", 0);

    if( !left_img.data || !right_img.data ){
        printf(" --(!) Error reading images \n"); return -1;
    }

    ros::Time::init();
    while(!ros::Time::isValid());
    ros::Time begin  = ros::Time::now();
    ros::Time end;

//----STEP 1.a: FAST feature detection
    // Ptr<FeatureDetector> detector;
    // detector = new DynamicAdaptedFeatureDetector ( new FastAdjuster(10,true), 5000, 10000, 10);
    // std::vector<cv::KeyPoint> left_keypoints;
    // std::vector<cv::KeyPoint> right_keypoints;

    // detector->detect(left_img, left_keypoints);
    // detector->detect(right_img, right_keypoints);

//----STEP 1.b: SURF feature detection
    SURF detector(400);
    std::vector<cv::KeyPoint> left_keypoints;
    std::vector<cv::KeyPoint> right_keypoints;
    Mat left_mask = cv::Mat::ones(left_img.size(), CV_8U);
    Mat right_mask = cv::Mat::ones(right_img.size(), CV_8U);

    detector(left_img, left_mask, left_keypoints);
    detector(right_img, right_mask, right_keypoints);

    end = ros::Time::now();
    std::cout << "Time spent on step 1: " << end - begin << std::endl;
    begin = ros::Time::now();

//----STEP 2.a: SIFT descriptors
//    Mat left_descriptors;
//    Mat right_descriptors;

//    SIFT extractor(0, 4, 0.03, 10, 1.6);
//    extractor(left_img, left_mask, left_keypoints, left_descriptors, true);
//    extractor(right_img, right_mask, right_keypoints, right_descriptors, true);

    //----STEP 2.B: SURF descriptors
    Mat left_descriptors;
    Mat right_descriptors;

    detector(left_img, left_mask, left_keypoints, left_descriptors, true);
    detector(right_img, right_mask, right_keypoints, right_descriptors, true);

    end = ros::Time::now();
    std::cout << "Time spent on step 2: " << end - begin << std::endl;
    begin = ros::Time::now();

//----STEP 3.a: Brute Force matching
//	vector< vector<DMatch> > matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
//	matcher->knnMatch( left_descriptors, right_descriptors, matches, 500 );

//----STEP 3.b: FLANN matching
	cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch( left_descriptors, right_descriptors, matches, 2 );

	end = ros::Time::now();
    std::cout << "Time spent on step 3: " << end - begin << std::endl;
    begin = ros::Time::now();

//----STEP 4: Refining using match only inside region
	//look whether the match is inside a defined area of the image
	//only 25% of maximum of possible distance
	double thresholdDist2 = 0.0625 * double(left_img.size().height*left_img.size().height + 
							left_img.size().width*left_img.size().width);
	//thresholdDist ^ 2 is used instead for speed purposes

	vector< cv::DMatch > good_matches2;
	good_matches2.reserve(matches.size());  
	for (size_t i = 0; i < matches.size(); ++i)
	{ 
	    for (int j = 0; j < matches[i].size(); j++)
	    {
        	Point2f from = left_keypoints[matches[i][j].queryIdx].pt;
        	Point2f to = right_keypoints[matches[i][j].trainIdx].pt;

	        //calculate local distance for each possible match
			double dist2 = (from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y);
	
        	//save as best match if local distance is in specified area and on same height
        	if (dist2 < thresholdDist2 && abs(from.y-to.y)<3)
        	{
	            good_matches2.push_back(matches[i][j]);
            	j = matches[i].size();
        	}
    	}
	}

	end = ros::Time::now();
    std::cout << "Time spent on step 4: " << end - begin << std::endl;
    begin = ros::Time::now();


//----STEP 5: Output the results
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
	    waitKey();
    }

    std::cout << some_good_matches[0].queryIdx << std::endl;
    std::cout << some_good_matches[0].trainIdx << std::endl;
    std::cout << some_good_matches[0].imgIdx << std::endl;
    std::cout << some_good_matches[0].distance << std::endl;

    cv::destroyWindow("Good Matches");
    return 0;

// TODO:	EPIPOLAR CONSTRAINT: VERTICAL COORDINATE MUST BE LOWER THAT A LIMIT (~1PX)
//    		DISPARITY CONSTRAINT: HORIZONTAL COORDINATE OF LEFT CAMERA MUST BE GREATER (BUT NOT GREATER THAT A CERTAIN LIMIT)
//			ORIENTATION CONSTRAINT: DIFFERENCE BETWEEN 2 ORIENTATIONS MUST BE WITHIN A CERTAIN LIMIT

//			DISPARITY = B*f/Z    ---> POR AQUÍ VA LA COSA

// NOTES: 	1)FLANN ES SIGNIFICATIVAMENTE MÁS RÁPIDO
    //		2)SURF NO ES MÁS RÁPIDO PERO A OJO PARECE MEJOR, SIN EMBARGO, DEVUELVE MUCHO MENOS PUNTOS
    //		3)SIFT PARECE DEMASIADO LENTO (1 con respecto a 0.24 de SURF)
    //		4)EL REFINADO ES MUY RÁPIDO, Y FUNCIONA MUY BIEN. DE MOMENTO NO ES NECESARIO MEJORARLO (EPIPOLAR...BLABLA)
    //		radiusMatch no funciona, hay muy claros outliers

    // EN TOTAL TARDA POCO MÁS DE 0.5 -> HABRÍA QUE TRABAJAR A 2FPS :(
}
