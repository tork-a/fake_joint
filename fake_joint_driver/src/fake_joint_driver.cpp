/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include "fake_joint_driver/fake_joint_driver.h"

#include <urdf/model.h>

#include <fstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fake_joint_driver");

FakeJointDriver::FakeJointDriver(const rclcpp::Node::SharedPtr& node)
{
  std::set<std::string> joint_set;
  std::map<std::string, double> start_position_map;

  // Read parameters
  use_description_ = node->declare_parameter("use_robot_description", true);
  include_joints_ = node->declare_parameter("include_joints", std::vector<std::string>());
  exclude_joints_ = node->declare_parameter("exclude_joints", std::vector<std::string>());

  std::vector<std::string> joint_names = node->declare_parameter("start_position.joints", std::vector<std::string>());
  std::vector<double> joint_values = node->declare_parameter("start_position.values", std::vector<double>());

  if (joint_names.size() != joint_values.size())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "start_position.joints and start_position.values must have the same size");
    return;
  }

  for (std::size_t i = 0; i < joint_names.size(); i++)
    start_position_map.emplace(joint_names.at(i), joint_values.at(i));

  for (auto it = start_position_map.begin(); it != start_position_map.end(); it++)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "start_position: " << it->first << ": " << it->second);
  }

  for (auto i = 0; i < include_joints_.size(); i++)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "include_joint[" << i << "]" << include_joints_[i]);
  }
  for (auto i = 0; i < exclude_joints_.size(); i++)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "exclude_joint[" << i << "]" << exclude_joints_[i]);
  }
  // Read all joints in robot_description
  if (use_description_)
  {
    std::string urdf_xml = node->declare_parameter("robot_description", std::string());
    urdf::Model urdf_model;
    if (urdf_model.initString(urdf_xml))
    {
      for (auto it = urdf_model.joints_.begin(); it != urdf_model.joints_.end(); it++)
      {
        urdf::Joint joint = *it->second;
        // remove fixed and unknown joints
        if (joint.type == urdf::Joint::FIXED || joint.type == urdf::Joint::UNKNOWN)
        {
          continue;
        }
        joint_set.insert(joint.name);
      }
    }
    else
    {
      RCLCPP_WARN(LOGGER, "We cannot find the parameter robot_description.");
    }
  }
  // Include joints into joint_set
  for (auto i = 0; i < include_joints_.size(); i++)
  {
    joint_set.insert(include_joints_[i]);
  }
  // Exclude joints in joint_set
  for (auto i = 0; i < exclude_joints_.size(); i++)
  {
    joint_set.erase(exclude_joints_[i]);
  }
  // Convert to vector (joint_names_)
  std::copy(joint_set.begin(), joint_set.end(), std::back_inserter(joint_names_));
  // Check the emptyness of joints
  if (joint_names_.size() == 0)
  {
    RCLCPP_ERROR(LOGGER, "No joints is specified. Please use include_joints parameters.");
    rclcpp::shutdown();
  }
  // Resize members
  cmd_dis.resize(joint_names_.size());
  act_dis.resize(joint_names_.size());
  act_vel.resize(joint_names_.size());
  act_eff.resize(joint_names_.size());
  op_mode.resize(joint_names_.size());
  joint_state_handles_.resize(joint_names_.size());
  joint_command_handles_.resize(joint_names_.size());
  joint_mode_handles_.resize(joint_names_.size());

  // Set start position
  for (auto it = start_position_map.begin(); it != start_position_map.end(); it++)
  {
    for (auto i = 0; i < joint_names_.size(); i++)
    {
      if (joint_names_[i] == it->first)
      {
        act_dis[i] = it->second;
        cmd_dis[i] = it->second;
      }
    }
  }

  // Create joint_state_interface
  for (int i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "joint[" << i << "]:" << joint_names_[i]);
    // Connect and register the joint_state_interface
    joint_state_handles_[i] =
        hardware_interface::JointStateHandle(joint_names_[i], &act_dis[i], &act_vel[i], &act_eff[i]);
    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::HW_RET_OK)
      throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());

    joint_command_handles_[i] = hardware_interface::JointCommandHandle(joint_names_[i], &cmd_dis[i]);
    if (register_joint_command_handle(&joint_command_handles_[i]) != hardware_interface::HW_RET_OK)
    {
      throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    }

    joint_mode_handles_[i] = hardware_interface::OperationModeHandle(joint_names_[i], &op_mode[i]);
    if (register_operation_mode_handle(&joint_mode_handles_[i]) != hardware_interface::HW_RET_OK)
    {
      throw std::runtime_error("unable to register " + joint_mode_handles_[i].get_name());
    }
  }
}

FakeJointDriver::~FakeJointDriver()
{
}

/**
 * @brief Update function to call all of the update function of motors
 */
void FakeJointDriver::update(void)
{
  // only do loopback
  act_dis = cmd_dis;
}
