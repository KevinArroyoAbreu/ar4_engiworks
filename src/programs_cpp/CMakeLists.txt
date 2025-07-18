cmake_minimum_required(VERSION 3.8)
project(programs_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(position_tracker REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# Other requirements for C and Cpp
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


# Executable 1
add_executable(prog1 src/program1.cpp)
ament_target_dependencies(prog1
  rclcpp
  moveit_ros_planning_interface
)

# Executable 2
add_executable(cubeRoutine src/cubeRoutine.cpp)
ament_target_dependencies(cubeRoutine
  rclcpp
  moveit_ros_planning_interface
  position_tracker
)

add_executable(cubeRoutineV2 src/cubeRoutineV2.cpp)
ament_target_dependencies(cubeRoutineV2
  rclcpp
  moveit_ros_planning_interface
  position_tracker
)


# Launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install targets
install(TARGETS prog1 cubeRoutine cubeRoutineV2
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip some linters until copyrights/licenses are added
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
