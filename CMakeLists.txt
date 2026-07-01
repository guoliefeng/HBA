cmake_minimum_required(VERSION 3.0.2)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread timer chrono filesystem system)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# Hierarchical Bundle Adjustment
add_executable(hba source/hba.cpp)
target_link_libraries(hba ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(visualize_map source/visualize.cpp)
target_link_libraries(visualize_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_preprocess source/map_data_processor.cpp)
target_link_libraries(map_preprocess ${PCL_LIBRARIES} Boost::filesystem Boost::system)

add_executable(merge_pcd source/merge_pcd.cpp)
target_link_libraries(merge_pcd ${PCL_LIBRARIES} Boost::filesystem Boost::system)

add_executable(loop_optimize source/loop_optimize.cpp)
target_link_libraries(loop_optimize ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(track_align source/track_align.cpp)
target_link_libraries(track_align ${PCL_LIBRARIES})

add_executable(gps_optimize source/gps_optimize.cpp)
target_link_libraries(gps_optimize gtsam)

add_executable(rebuild_map_no_crop source/rebuild_map_no_crop.cpp)
target_link_libraries(rebuild_map_no_crop ${PCL_LIBRARIES})
