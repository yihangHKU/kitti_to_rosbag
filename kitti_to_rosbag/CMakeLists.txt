cmake_minimum_required(VERSION 2.8.3)
project(kitti_to_rosbag)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs)
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 
set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/kitti_parser.cpp
  src/kitti_ros_conversions.cpp
  src/kitti_common.cpp
)

############
# BINARIES #
############
cs_add_executable(kitti_live_node
  src/kitti_live_node.cpp
)
target_link_libraries(kitti_live_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(kitti_rosbag_converter
  src/kitti_rosbag_converter.cpp
)
target_link_libraries(kitti_rosbag_converter ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(tiny_disp_view
  src/tiny_disp_view.cpp
)
target_link_libraries(tiny_disp_view ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(kitti_static_filter
  src/kitti_static_label_filter.cpp
)
target_link_libraries(kitti_static_filter ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(write_dyn_labels
  src/kitti_dyn_points_extract.cpp
)
target_link_libraries(write_dyn_labels ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(pose_file_transfer
  src/pose_file_transfer.cpp
)
target_link_libraries(pose_file_transfer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(kittibbox_transfer
  src/tracking_transfer_kittibbox.cpp
)
target_link_libraries(kittibbox_transfer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(kitti_tracking_statistics
  src/kitti_tracking_statistics.cpp
)
target_link_libraries(kitti_tracking_statistics ${PROJECT_NAME} ${OpenCV_LIBRARIES})


cs_add_executable(record_pose_vicon
  src/record_pose_vicon.cpp
)
target_link_libraries(record_pose_vicon ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(extract_bag
  src/extract_from_bag.cpp
)
target_link_libraries(extract_bag ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(write_static_bin
  src/kitti_static_points_extract.cpp
)
target_link_libraries(write_static_bin ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(timestamp_correct
  src/bag_timestamp_correct.cpp
)
target_link_libraries(timestamp_correct ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(write_pcd
  src/write_pcd.cpp
)
target_link_libraries(write_pcd ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(project_point
  src/pcl_project2img.cpp
)
target_link_libraries(project_point ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(nus_point_transfer
  src/nus_point_transfer.cpp
)
target_link_libraries(nus_point_transfer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(format_transfer
  src/tracking_format_transfer.cpp
)
target_link_libraries(format_transfer ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(gt_revision
  src/tracking_gt_revision.cpp
)
target_link_libraries(gt_revision ${PROJECT_NAME} ${OpenCV_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
