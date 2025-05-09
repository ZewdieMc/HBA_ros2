cmake_minimum_required(VERSION 3.8)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)  # Add this line



include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(hba src/hba.cpp)
target_link_libraries(hba ${PCL_LIBRARIES} gtsam)
ament_target_dependencies(hba rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros PCL Eigen3 pcl_conversions rosbag2_cpp )

add_executable(visualize_map src/visualize.cpp)
target_link_libraries(visualize_map ${PCL_LIBRARIES})
ament_target_dependencies(visualize_map rclcpp std_msgs visualization_msgs PCL Eigen3 pcl_conversions visualization_msgs tf2 tf2_ros rosbag2_cpp)

# add_executable(calculate_MME src/calculate_MME.cpp)
# target_link_libraries(calculate_MME ${PCL_LIBRARIES} gtsam)
# ament_target_dependencies(calculate_MME rclcpp std_msgs geometry_msgs tf2 tf2_ros PCL Eigen3 pcl_conversions)

install(TARGETS
  hba
  visualize_map
  # calculate_MME
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
