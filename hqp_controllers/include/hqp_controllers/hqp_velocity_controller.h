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
 * This class forwards the command signal down to a set of joints.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \tparam T Type implementing the JointCommandInterface.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
 */
template <class T>
class ForwardJointGroupCommandController: public controller_interface::Controller<T>
{
public:
  ForwardJointGroupCommandController() { commands_.clear(); }
  ~ForwardJointGroupCommandController() {sub_command_.shutdown();}

  bool init(T* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();
    
    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));  
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }
    
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ForwardJointGroupCommandController::commandCB, this);
    return true;
  }

  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period) 
  {
    for(unsigned int i=0; i<n_joints_; i++)
    {  joints_[i].setCommand(commands_[i]);  }
  }

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  std::vector< double > commands_;
  unsigned int n_joints_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg) 
  {
    if(msg->data.size()!=n_joints_)
    { 
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return; 
    }
    for(unsigned int i=0; i<n_joints_; i++)
    {  commands_[i] = msg->data[i];  }
  }
};

}

#endif
