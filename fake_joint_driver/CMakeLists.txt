cmake_minimum_required(VERSION 3.10.2)
project(fake_joint_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

include_directories(include)

add_executable(fake_joint_driver_node
  src/fake_joint_driver_node.cpp src/fake_joint_driver.cpp)

ament_target_dependencies(fake_joint_driver_node
    rclcpp
    controller_manager
    hardware_interface
    controller_interface
    urdf
)

install(TARGETS fake_joint_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config robot test
  DESTINATION share)

# catkin_install_python(
#   PROGRAMS test/test_fake_joint_driver.py
#   DESTINATION share/test)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

# if(CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)
#   set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
#   set(ROSLINT_PYTHON_OPTS "--max-line-length=180")
#   roslint_cpp(src/fake_joint_driver.cpp src/fake_joint_driver_node.cpp)
#   # add_dependencies(tests roslint)
#   file(GLOB LAUNCH_FILES launch/*.launch)
#   foreach(LAUNCH_FILE ${LAUNCH_FILES})
#     message(STATUS "Testing ${LAUNCH_FILE}")
#     roslaunch_add_file_check(${LAUNCH_FILE})
#   endforeach()

#   add_rostest(test/fake_joint_driver.test)
# endif()
ament_export_dependencies(rclcpp)
ament_package()
