cmake_minimum_required(VERSION 3.8)
project(bag_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(rosbag2_cpp  REQUIRED)
find_package(turtlesim REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(application_data_object_management_msgs REQUIRED)
find_package(adom REQUIRED)
find_package(CURL REQUIRED)
message(STATUS "Using CURL in ${CURL_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(image_bag_reader src/image_bag_reader.cpp)
ament_target_dependencies(image_bag_reader rclcpp rosbag2_cpp image_transport sensor_msgs cv_bridge application_data_object_management_msgs adom)
target_link_libraries(image_bag_reader adom curl ${OpenCV_LIBS})

add_executable(image_bag_reader_adom src/image_bag_reader_adom.cpp)
ament_target_dependencies(image_bag_reader_adom rclcpp rosbag2_cpp image_transport sensor_msgs cv_bridge application_data_object_management_msgs adom)
target_link_libraries(image_bag_reader_adom curl adom ${OpenCV_LIBS})

add_executable(roi_bag_reader src/roi_bag_reader.cpp)
ament_target_dependencies(roi_bag_reader rclcpp rosbag2_cpp sensor_msgs visualization_msgs tier4_perception_msgs)


install(TARGETS
  image_bag_reader
  image_bag_reader_adom
  roi_bag_reader
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
