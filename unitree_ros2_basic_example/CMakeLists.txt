cmake_minimum_required(VERSION 3.8)
project(unitree_ros2_basic_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(g1_basic_low_state src/g1_basic_low_state.cpp)
ament_target_dependencies(g1_basic_low_state rclcpp std_msgs unitree_hg)

add_executable(g1_move_ankle src/g1_move_ankle.cpp)
ament_target_dependencies(g1_move_ankle rclcpp std_msgs unitree_hg)

install(TARGETS
  g1_basic_low_state
  g1_move_ankle
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
