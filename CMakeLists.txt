cmake_minimum_required(VERSION 3.8)
project(swerve_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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

  ament_add_pytest_test(kinematics test/test_kinematics.py)
  ament_add_pytest_test(motion test/test_motion.py)
  ament_add_pytest_test(model test/test_model.py)
  ament_add_pytest_test(controller test/test_controller.py)
  ament_add_pytest_test(launch test/test_launch.py)
  ament_add_pytest_test(runner test/test_runner.py)
endif()

ament_package()
