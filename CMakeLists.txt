cmake_minimum_required(VERSION 3.5)
project(pc2_to_laserscan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pc2_to_laserscan_cpp src/pc2_to_laserscan.cpp)
ament_target_dependencies(pc2_to_laserscan_cpp rclcpp sensor_msgs std_msgs)


install(TARGETS
  pc2_to_laserscan_cpp
  DESTINATION lib/${PROJECT_NAME}
)
  

  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)




ament_package()
