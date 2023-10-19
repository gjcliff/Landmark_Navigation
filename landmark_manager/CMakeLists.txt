cmake_minimum_required(VERSION 3.8)
project(landmark_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/SaveLandmark.srv"
  "srv/NavigateToLandmark.srv"
  LIBRARY_NAME ${PROJECT_NAME})
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(landmark_manager src/landmark_manager.cpp)

ament_target_dependencies(landmark_manager
  rclcpp
  rclcpp_action
  visualization_msgs
  std_srvs
  nav2_msgs
  yaml-cpp
  ament_index_cpp)

target_link_libraries(landmark_manager yaml-cpp "${cpp_typesupport_target}")

install(TARGETS
  landmark_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  foxglove
  DESTINATION share/landmark_manager/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
