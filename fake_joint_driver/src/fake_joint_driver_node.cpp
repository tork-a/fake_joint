/**
 * @file fake_joint_driver_node.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * Device driver node to fake loopback joints.
 */
#include <controller_manager/controller_manager.hpp>
#include <fake_joint_driver/fake_joint_driver.h>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Main function
 */
int main(int argc, char **argv) {
  // Init ROS node
  rclcpp::init(argc, argv);
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  auto node = rclcpp::Node::make_shared("fake_joint_driver_node");

  // Create hardware interface
  auto robot = std::make_shared<FakeJointDriver>(node, non_ros_args.at(1));
  // Set spin ratge
  rclcpp::Rate rate(1.0 / rclcpp::Duration(0.010).seconds());
  auto executor = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor->add_node(node);
  // Connect to controller manager
  controller_manager::ControllerManager cm(robot, executor);

  while (rclcpp::ok())
  {
    robot->update();
    cm.update();
    rate.sleep();
  }

  return 0;
}
