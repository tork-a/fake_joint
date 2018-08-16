cmake_minimum_required(VERSION 2.8.3)
project(fake_joint_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  roslaunch
  rostest
  urdf
  xacro
)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS hardware_interface roscpp
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_joint_driver_node
  src/fake_joint_driver_node.cpp src/fake_joint_driver.cpp)

target_link_libraries(fake_joint_driver_node
  ${catkin_LIBRARIES} )

install(TARGETS fake_joint_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config robot test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS test/test_fake_joint_driver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
  set(ROSLINT_PYTHON_OPTS "--max-line-length=180")
  roslint_cpp(src/fake_joint_driver.cpp src/fake_joint_driver_node.cpp)
  # add_dependencies(tests roslint)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/fake_joint_driver.test)
endif()
