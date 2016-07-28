/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
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
  std::vector<std::string> joint_names;
  ros::NodeHandle nh;

  // Read all joints in robot_description
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description")) {
    ROS_ERROR("Failed to parse robot_description");
    exit(0);
  }
  std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator iter;
  for (iter=urdf_model.joints_.begin(); iter!=urdf_model.joints_.end(); iter++) {
    urdf::Joint joint = *iter->second;
    // remove fixed and unknown joints
    if (joint.type != urdf::Joint::FIXED && joint.type != urdf::Joint::UNKNOWN) {
      joint_names.push_back(joint.name);
    }
  }
  // resize members
  int joint_num = joint_names.size();
  if (joint_num == 0) {
    ROS_ERROR("No valid joints found");
    exit(0);
  }
  cmd_dis.resize(joint_num);
  act_dis.resize(joint_num);
  act_vel.resize(joint_num);
  act_eff.resize(joint_num);

  for (int i = 0; i < joint_num; i++)
  {
    ROS_INFO_STREAM("joint: " << joint_names[i]);
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &act_dis[i], &act_vel[i], &act_eff[i]);
    joint_state_interface.registerHandle(state_handle);

    // connect and register the position joint interface
    hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_names[i]), &cmd_dis[i]);
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

