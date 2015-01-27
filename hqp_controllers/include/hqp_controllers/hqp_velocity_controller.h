#ifndef HQP_VELOCITY_CONTROLLER_H
#define HQP_VELOCITY_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <hqp_controllers_msgs/VisualizeTaskObjects.h>
#include <realtime_tools/realtime_publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <hqp_controllers/task_manager.h>
#include <hqp_controllers_msgs/SetTask.h>
#include <hqp_controllers_msgs/SetTaskObject.h>

namespace hqp_controllers
{
#define TASK_OBJ_PUBLISH_RATE 50 //The rate to publish task object geometries

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
         ros::ServiceServer set_task_srv_;
     ros::ServiceServer set_task_obj_srv_;
          ros::ServiceServer vis_t_obj_srv_;
          TaskManager task_manager_;
     boost::mutex lock_;

     realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> vis_t_obj_pub_;
     ros::Time last_publish_time_;
     double publish_rate_;
     Eigen::VectorXi vis_ids_; ///< only task object geometries with ids in vis_ids_ will be published
     bool active_;

     ///////////////
     // CALLBACKS //
     ///////////////

     void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
     bool setTask(hqp_controllers_msgs::SetTask::Request & req, hqp_controllers_msgs::SetTask::Response &res);
       bool setTaskObject(hqp_controllers_msgs::SetTaskObject::Request & req, hqp_controllers_msgs::SetTaskObject::Response &res);
     bool visualizeTaskObjects(hqp_controllers_msgs::VisualizeTaskObjects::Request & req, hqp_controllers_msgs::VisualizeTaskObjects::Response &res);
  };


} //end namespace hqp_controllers

#endif
