cmake_minimum_required(VERSION 3.5)
project(turtlebot_line_follower)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

install(
  PROGRAMS
    src/line_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
