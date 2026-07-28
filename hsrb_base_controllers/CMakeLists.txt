cmake_minimum_required(VERSION 3.5)
project(hsrb_base_controllers)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tmc_realtime_tools REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_modules SHARED
  src/${PROJECT_NAME}/command_subscriber.cpp
  src/${PROJECT_NAME}/omni_base_control_method.cpp
  src/${PROJECT_NAME}/omni_base_input_odometry.cpp
  src/${PROJECT_NAME}/omni_base_joint_controller.cpp
  src/${PROJECT_NAME}/omni_base_odometry.cpp
  src/${PROJECT_NAME}/omni_base_state.cpp
  src/${PROJECT_NAME}/utils.cpp
)
target_compile_features(${PROJECT_NAME}_modules PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME}_modules
  PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME}_modules
  angles
  control_msgs
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  realtime_tools
  tf2_msgs
  tmc_realtime_tools
  tmc_utils
  trajectory_msgs
  urdf
)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/omni_base_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_modules)
ament_target_dependencies(${PROJECT_NAME} controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(controller_interface controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_modules
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gmock(command_subscriber_test test/command_subscriber-test.cpp)
  target_link_libraries(command_subscriber_test ${PROJECT_NAME})

  ament_add_gtest(filter_test test/filter-test.cpp)
  target_link_libraries(filter_test ${PROJECT_NAME})

  ament_add_gtest(omni_base_control_method_test test/omni_base_control_method-test.cpp)
  target_link_libraries(omni_base_control_method_test ${PROJECT_NAME})

  ament_add_gtest(omni_base_controller_test test/omni_base_controller-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
                  TIMEOUT 150)
  target_link_libraries(omni_base_controller_test ${PROJECT_NAME})

  ament_add_gtest(omni_base_input_odometry_test test/omni_base_input_odometry-test.cpp)
  target_link_libraries(omni_base_input_odometry_test ${PROJECT_NAME})

  find_package(Boost REQUIRED COMPONENTS thread)
  ament_add_gtest(omni_base_joint_controller_test test/omni_base_joint_controller-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(omni_base_joint_controller_test ${PROJECT_NAME} Boost::thread)

  ament_add_gtest(omni_base_joint_controller_acc_limit_test test/omni_base_joint_controller_acc_limit-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(omni_base_joint_controller_acc_limit_test ${PROJECT_NAME})

  ament_add_gtest(omni_base_odometry_test test/omni_base_odometry-test.cpp)
  target_link_libraries(omni_base_odometry_test ${PROJECT_NAME})

  ament_add_gtest(omni_base_state_test test/omni_base_state-test.cpp)
  target_link_libraries(omni_base_state_test ${PROJECT_NAME})

  ament_add_gtest(twin_caster_drive_test test/twin_caster_drive-test.cpp)
  target_link_libraries(twin_caster_drive_test ${PROJECT_NAME})

  ament_add_gtest(utils_test test/utils-test.cpp)
  target_link_libraries(utils_test ${PROJECT_NAME})
endif()

ament_export_dependencies(
  controller_interface
  control_msgs
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  tf2_msgs
  trajectory_msgs
  urdf
)
ament_export_include_directories(
  include
)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_modules)
ament_package()
