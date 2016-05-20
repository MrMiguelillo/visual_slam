cmake_minimum_required(VERSION 2.8.3)
project(tfg)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
  uvc_camera
)
find_package( OpenCV REQUIRED )

add_message_files(
	FILES
	dmatch.msg
  dmatchArray.msg
  keypoint.msg
  keypointArray.msg
  pointsMatched.msg
  pointsMatchedArray.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

#add_executable( feature_matching src/feature_matching.cpp )
#target_link_libraries ( feature_matching ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)

#add_executable( calibrate_single_cam src/calibrate_single_cam.cpp )
#target_link_libraries ( calibrate_single_cam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable( offline_feature_matching src/offline_feature_matching.cpp )
target_link_libraries ( offline_feature_matching ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)

add_executable( image_feeder src/image_feeder.cpp )
target_link_libraries ( image_feeder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable( online_feature_matching src/online_feature_matching.cpp )
target_link_libraries ( online_feature_matching ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)
add_dependencies(online_feature_matching tfg_gencpp)

add_executable( dmatch_publish_test src/dmatch_publish_test.cpp )
target_link_libraries ( dmatch_publish_test ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)
add_dependencies(dmatch_publish_test tfg_gencpp)