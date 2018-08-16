/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include <ros/ros.h>
#include <urdf/model.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "fake_joint_driver/fake_joint_driver.h"

FakeJointDriver::FakeJointDriver(void)
{
  ros::NodeHandle pnh("~");
  std::set<std::string> joint_set;

  // Read parameters
  pnh.param<bool>("use_robot_description", use_description_, true);
  pnh.getParam("include_joints", include_joints_);
  pnh.getParam("exclude_joints", exclude_joints_);

  for (auto i=0; i<include_joints_.size(); i++)
  {
    ROS_DEBUG_STREAM("include_joint[" << i << "]" << include_joints_[i]);
  }
  for (auto i=0; i<exclude_joints_.size(); i++)
  {
    ROS_DEBUG_STREAM("exclude_joint[" << i << "]" << exclude_joints_[i]);
  }
  // Read all joints in robot_description
  if (use_description_)
  {
    urdf::Model urdf_model;
    if (urdf_model.initParam("robot_description"))
    {
      for (auto it=urdf_model.joints_.begin(); it!=urdf_model.joints_.end(); it++)
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
      ROS_WARN("We cannot find the parameter robot_description.");    
    }
  }
  // Include joints into joint_set
  for (auto i=0; i< include_joints_.size(); i++)
  {
    joint_set.insert(include_joints_[i]);
  }
  // Exclude joints in joint_set
  for (auto i=0; i< exclude_joints_.size(); i++)
  {
    joint_set.erase(exclude_joints_[i]);
  }
  // Convert to vector (joint_names_)
  std::copy(joint_set.begin(), joint_set.end(), std::back_inserter(joint_names_));
  // Check the emptyness of joints
  if (joint_names_.size() == 0) {
    ROS_ERROR("No joints is specified. Please use include_joints parameters.");
    ros::shutdown();
  }
  // Resize members
  cmd_dis.resize(joint_names_.size());
  act_dis.resize(joint_names_.size());
  act_vel.resize(joint_names_.size());
  act_eff.resize(joint_names_.size());

  // Create joint_state_interface and position_joint_interface
  for (int i = 0; i< joint_names_.size(); i++)
  {
    ROS_INFO_STREAM("joint[" << i << "]:" << joint_names_[i]);
    // Connect and register the joint_state_interface
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &act_dis[i], &act_vel[i], &act_eff[i]);
    joint_state_interface.registerHandle(state_handle);

    // Connect and register the position_joint_interface
    hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_names_[i]), &cmd_dis[i]);
    position_joint_interface.registerHandle(pos_handle);
  }
  registerInterface(&joint_state_interface);
  registerInterface(&position_joint_interface);
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

