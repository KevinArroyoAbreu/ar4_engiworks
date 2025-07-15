cmake_minimum_required(VERSION 3.5)
project(position_tracker)


# Find dependencies first
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Then generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPosition.srv"
  DEPENDENCIES std_msgs sensor_msgs
)



ament_export_dependencies(rosidl_default_runtime)

ament_package()
