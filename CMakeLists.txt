cmake_minimum_required(VERSION 3.20)
project(swerve_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCommand.msg" "msg/ChassisState.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

# rosidl owns the Python namespace; install our modules alongside generated messages.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY ${PROJECT_NAME}/ DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py" PATTERN "__init__.py" EXCLUDE)
install(DIRECTORY config launch urdf worlds docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(suite chassis kinematics model launch truth controller)
    ament_add_pytest_test(${suite} test/test_${suite}.py)
  endforeach()
endif()
ament_package()
