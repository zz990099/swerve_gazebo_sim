cmake_minimum_required(VERSION 3.8)
project(swerve_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(guarded_joint_controller SHARED src/guarded_joint_controller.cpp)
target_compile_features(guarded_joint_controller PUBLIC cxx_std_17)
target_include_directories(guarded_joint_controller PRIVATE include)
ament_target_dependencies(guarded_joint_controller controller_interface pluginlib realtime_tools std_msgs)
pluginlib_export_plugin_description_file(controller_interface guarded_joint_controller.xml)
install(TARGETS guarded_joint_controller LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch urdf worlds docs
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(hardware_interface REQUIRED)
  add_executable(endpoint_guard_tests test/endpoint_guard_tests.cpp)
  target_compile_features(endpoint_guard_tests PRIVATE cxx_std_17)
  target_include_directories(endpoint_guard_tests PRIVATE include)
  target_compile_options(endpoint_guard_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME endpoint_guard COMMAND endpoint_guard_tests)
  add_executable(controller_lifecycle_tests test/controller_lifecycle_tests.cpp)
  target_compile_features(controller_lifecycle_tests PRIVATE cxx_std_17)
  ament_target_dependencies(controller_lifecycle_tests controller_interface pluginlib std_msgs rclcpp hardware_interface)
  add_test(NAME controller_lifecycle COMMAND controller_lifecycle_tests)
  set_tests_properties(controller_lifecycle PROPERTIES TIMEOUT 30)

  ament_add_pytest_test(endpoint_config test/test_endpoint_config.py)
  ament_add_pytest_test(kinematics test/test_kinematics.py)
  ament_add_pytest_test(motion test/test_motion.py)
  ament_add_pytest_test(model test/test_model.py)
  ament_add_pytest_test(controller test/test_controller.py)
  ament_add_pytest_test(launch test/test_launch.py)
  ament_add_pytest_test(runner test/test_runner.py)
  ament_add_pytest_test(truth test/test_truth.py)
endif()

ament_package()
