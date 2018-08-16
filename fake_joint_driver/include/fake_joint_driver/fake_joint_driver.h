/**
 * @file fake_joint_driver.h
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class FakeJointDriver : public hardware_interface::RobotHW
{
private:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  std::vector<double> cmd_dis;
  std::vector<double> act_dis;
  std::vector<double> act_vel;
  std::vector<double> act_eff;

  std::vector<std::string> joint_names_;
  bool use_description_;
  std::vector<std::string> include_joints_;
  std::vector<std::string> exclude_joints_;

public:
  FakeJointDriver(void);
  ~FakeJointDriver();
  void update(void);
};
