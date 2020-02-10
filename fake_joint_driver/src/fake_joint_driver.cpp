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

template <typename ParameterT>
bool getParameterAsMap(const rclcpp::Node::SharedPtr& node, const std::string& prefix,
                       const std::array<std::string, 2>& first_second_names,
                       std::map<std::string, ParameterT>& param_map)
{
  /*
    start_position:
      joints:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7
      values:
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
   */
  rclcpp::Parameter param_keys;
  rclcpp::Parameter param_values;
  std::string current_parameter = prefix + "." + first_second_names.at(0);
  if (node->get_parameter(current_parameter, param_keys))
  {
    current_parameter = prefix + "." + first_second_names.at(1);
    if (node->get_parameter(current_parameter, param_values))
    {
      if (param_keys.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        RCLCPP_ERROR(LOGGER, "Parameter have to be array of strings");
        return false;
      }
      std::vector<std::string> keys = param_keys.as_string_array();
      std::vector<ParameterT> values = param_values.get_value<std::vector<ParameterT>>();
      if (keys.size() != values.size())
      {
        // TODO(JafarAbdi): Complete :D
        RCLCPP_ERROR_STREAM(LOGGER, "The parameter");
        return false;
      }
      for (std::size_t index = 0; index < keys.size(); index++)
      {
        param_map.emplace(keys.at(index), values.at(index));
      }
      return true;
    }
  }

  RCLCPP_ERROR_STREAM(LOGGER, "Can't fine parameter " << current_parameter);
  return false;
}

FakeJointDriver::FakeJointDriver(const rclcpp::Node::SharedPtr& node, const std::string& robot_description_file_path)
{
  std::set<std::string> joint_set;
  std::map<std::string, double> start_position_map;

  // Read parameters
  use_description_ = node->declare_parameter("use_robot_description", true);
  include_joints_ = node->declare_parameter("include_joints", std::vector<std::string>());
  exclude_joints_ = node->declare_parameter("exclude_joints", std::vector<std::string>());

  //
  node->declare_parameter("start_position.joints");
  node->declare_parameter("start_position.values");

  getParameterAsMap(node, "start_position", { "joints", "values" }, start_position_map);

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
    std::string urdf_xml;
    // Load robot description from file
    try
    {
      std::ifstream in(robot_description_file_path, std::ios::in | std::ios::binary);
      if (in)
      {
        in.seekg(0, std::ios::end);
        urdf_xml.resize(in.tellg());
        in.seekg(0, std::ios::beg);
        in.read(&urdf_xml[0], urdf_xml.size());
        in.close();
      }
      else
      {
        throw std::system_error(errno, std::system_category(),
                                "Failed to open URDF file: " + robot_description_file_path);
      }
    }
    catch (const std::runtime_error& err)
    {
      RCLCPP_FATAL(LOGGER, "%s", err.what());
      throw;
    }

    urdf::Model urdf_model;
    // TODO(JafarAbdi): Test
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

  // Create joint_state_interface and position_joint_interface
  for (int i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "joint[" << i << "]:" << joint_names_[i]);
    // Connect and register the joint_state_interface
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &act_dis[i], &act_vel[i], &act_eff[i]);
    if (register_joint_state_handle(&state_handle) != hardware_interface::HW_RET_OK)
      throw std::runtime_error("unable to register " + state_handle.get_name());
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
