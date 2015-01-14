#ifndef HQP_VELOCITY_CONTROLLER_H
#define HQP_VELOCITY_CONTROLLER_H

#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>

namespace hqp_controllers
{

  /**
   * \brief Forward command controller for a set of joints.
   *
   * This class ... TODO ...
   */
  class HQPVelocityController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:

    HQPVelocityController();
    ~HQPVelocityController();

    std::vector< std::string > joint_names_;
    std::vector< hardware_interface::JointHandle > joints_;
    std::vector< double > commands_;
    unsigned int n_joints_;

    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period); 

  private:

    ros::Subscriber sub_command_;
    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); 
  };


} //end namespace hqp_controllers

#endif
