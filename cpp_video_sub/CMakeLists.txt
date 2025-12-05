cmake_minimum_required(VERSION 3.8)
project(cpp_video_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(main_cam_node 
  src/main_ros.cpp 
  src/chad_mod.cpp
)

ament_target_dependencies(main_cam_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(main_cam_node ${OpenCV_LIBS})

install(TARGETS
  main_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()