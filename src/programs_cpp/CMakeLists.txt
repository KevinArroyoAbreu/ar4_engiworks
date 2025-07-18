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
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)



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

add_executable(cubeRoutineV3 src/cubeRoutineV3.cpp)
ament_target_dependencies(cubeRoutineV3
  rclcpp
  moveit_ros_planning_interface
  position_tracker
  move_it_task_constructor_core
  rclcpp
)
target_include_directories(cubeRoutineV3 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# Launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install targets
install(TARGETS prog1 cubeRoutine cubeRoutineV2 cubeRoutineV3
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
