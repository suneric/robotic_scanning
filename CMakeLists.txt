cmake_minimum_required(VERSION 2.8.3)
project(robotic_scanning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(iiwa_ros REQUIRED)
find_package(iiwa_msgs REQUIRED)

find_package(realsense2 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree features surface keypoints octree recognition sample_consensus search segmentation stereo tracking visualization)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp iiwa_ros iiwa_msgs
  DEPENDS PCL OpenCV
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${iiwa_ros_INCLUDE_DIRS}
${realsense2_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(TARGET_H
    src/scanner/rs_scanner.h
    src/viewer/pcl_viewer.h
    src/util/geometry.h
    src/util/file_io.h
    src/util/config.h
    src/robot/iiwa_ros.h
    src/robot/iiwa_trajectory.h
    src/robot/iiwa_util.h
    src/robot/command/iiwa_command.h
    src/robot/command/joint_position_command.h
    src/robot/service/time_to_destination.h
    src/robot/state/cartesian_pose_state.h
    src/robot/state/joint_position_state.h
    )

set(TARGET_SRC
    src/scanner/rs_scanner.cpp
    src/main.cpp
    src/viewer/pcl_viewer.cpp
    src/util/geometry.cpp
    src/util/file_io.cpp
    src/util/config.cpp
    src/robot/iiwa_trajectory.cpp
    )

add_executable(${PROJECT_NAME}_exe ${TARGET_H} ${TARGET_SRC})
set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME scan3d PREFIX "")
add_dependencies(${PROJECT_NAME}_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_exe
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${realsense2_LIBRARY}
${iiwa_ros_LIBRARIES}
${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_exe
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
