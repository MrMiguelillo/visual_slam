cmake_minimum_required(VERSION 2.8.3)
project(tfg)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  uvc_camera
)
find_package( OpenCV REQUIRED )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

add_executable( feature_matching src/feature_matching.cpp )
target_link_libraries ( feature_matching ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)

add_executable( offline_feature_matching src/offline_feature_matching.cpp )
target_link_libraries ( offline_feature_matching ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)

add_executable( calibrate_single_cam src/calibrate_single_cam.cpp )
target_link_libraries ( calibrate_single_cam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} opencv_nonfree)