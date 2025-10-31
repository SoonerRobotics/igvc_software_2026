cmake_minimum_required(VERSION 3.8)
project(igvc_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(igvc REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} /usr/include/opencv4)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# local package include directories
include_directories(include)

# creating executables
add_executable(vision_test src/main.cpp src/pipeline.cpp)
ament_target_dependencies(vision_test igvc rclcpp std_msgs nav_msgs sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(vision_test ${OpenCV_LIBS} opencv_core opencv_imgproc opencv_highgui)

# installing executables
install(TARGETS
   vision_test
   DESTINATION lib/${PROJECT_NAME}
 )


ament_package()
