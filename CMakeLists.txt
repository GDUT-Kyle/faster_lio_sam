cmake_minimum_required(VERSION 3.0.2)
project(faster_lio_sam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_BUILD_TYPE Release)

set(CUSTOM_TBB_DIR "${PROJECT_SOURCE_DIR}/thirdparty/tbb2018_20170726oss")

if (CUSTOM_TBB_DIR)
    set(TBB2018_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2018_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB2018_INCLUDE_DIR})
    link_directories(${TBB2018_LIBRARY_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  livox_ros_driver
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(Eigen3 3.3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  merge_cloud.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES faster_lio_sam
 CATKIN_DEPENDS cv_bridge geometry_msgs livox_ros_driver message_generation nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(livox_repub src/livox_repub.cpp)
add_dependencies(livox_repub ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(livox_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(combineMultiLivox src/combineMultiLivox.cpp)
# add_dependencies(combineMultiLivox ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(combineMultiLivox ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ImageProjection src/ImageProjection.cpp)
add_dependencies(ImageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ImageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(imuPreintegration src/imuPreintegration.cpp)
add_dependencies(imuPreintegration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(fusionOptimization src/fusionOptimization.cpp)
add_dependencies(fusionOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(fusionOptimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} tbb glog)

# add_executable(poseOptimization src/poseOptimization.cpp)
# add_dependencies(poseOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(poseOptimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

