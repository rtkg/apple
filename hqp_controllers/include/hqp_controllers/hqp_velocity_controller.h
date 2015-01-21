#ifndef HQP_VELOCITY_CONTROLLER_H
#define HQP_VELOCITY_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <hqp_controllers/task_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <hqp_controllers_msgs/SetTaskObject.h>

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
    boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints_;

    std::vector< double > commands_;
    unsigned int n_joints_;

    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period); 

  private:

    ros::Subscriber sub_command_;
    ros::ServiceServer set_task_obj_srv_;
    TaskManager task_manager_;
    boost::mutex lock_;

    ///////////////
    // CALLBACKS //
    ///////////////

    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
    bool setTaskObject(hqp_controllers_msgs::SetTaskObject::Request & req, hqp_controllers_msgs::SetTaskObject::Response &res);
  };


} //end namespace hqp_controllers

#endif
