cmake_minimum_required(VERSION 3.8)
project(roi_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(adom REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(application_data_object_management_msgs REQUIRED)
find_package(CURL REQUIRED)


# include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(simple_roi_reader_dds src/simple_roi_reader_dds.cpp)
ament_target_dependencies(simple_roi_reader_dds rclcpp sensor_msgs cv_bridge image_transport tier4_perception_msgs OpenCV adom)
target_link_libraries(simple_roi_reader_dds adom curl ${OpenCV_LIBS})

add_executable(simple_roi_reader_adom src/simple_roi_reader_adom.cpp)
ament_target_dependencies(simple_roi_reader_adom rclcpp sensor_msgs cv_bridge image_transport tier4_perception_msgs application_data_object_management_msgs OpenCV adom)
target_link_libraries(simple_roi_reader_adom adom curl ${OpenCV_LIBS})

install(TARGETS
  simple_roi_reader_dds
  simple_roi_reader_adom
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
