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
#include <realtime_tools/realtime_publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <hqp_controllers/task_manager.h>
#include <hqp_controllers_msgs/SetTasks.h>
#include <hqp_controllers_msgs/SetTaskLinks.h>
#include <hqp_controllers_msgs/TaskStatuses.h>
#include <hqp_controllers_msgs/VisualizeTaskGeometries.h>
#include <hqp_controllers_msgs/ActivateHQPControl.h>
#include <hqp_controllers_msgs/RemoveTasks.h>
#include <hqp_controllers_msgs/RemoveTaskLinks.h>

namespace hqp_controllers
{
#define PUBLISH_RATE 50 //The rate to publish task geometries

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
    std::vector< hardware_interface::JointHandle  > joints_;
    KDL::Tree k_tree_;

    Eigen::VectorXd commands_;
    unsigned int n_joints_;

    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

private:

    ros::ServiceServer set_tasks_srv_;
    //ros::ServiceServer remove_tasks_srv_;
    ros::ServiceServer activate_hqp_control_srv_;
    //ros::ServiceServer reset_hqp_control_srv_;
    //ros::ServiceServer vis_t_link_srv_;
    TaskManager task_manager_;
    boost::mutex lock_;

    realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> vis_t_obj_pub_;
    realtime_tools::RealtimePublisher<hqp_controllers_msgs::TaskStatuses> t_statuses_pub_;
    ros::Time last_publish_time_;
    //double publish_rate_;
    //Eigen::VectorXi vis_ids_; ///< only task object geometries with ids in vis_ids_ will be published
    bool active_;

    //**Helper function to read joint limits from the parameter server and generate the corresponding tasks.*/
   // bool jointLimitsParser(ros::NodeHandle &n);

    ///////////////
    // CALLBACKS //
    ///////////////


    bool activateHQPControl(hqp_controllers_msgs::ActivateHQPControl::Request & req, hqp_controllers_msgs::ActivateHQPControl::Response &res);
   // bool resetHQPControl(std_srvs::Empty::Request & req, std_srvs::Empty::Response &res);
    bool setTasks(hqp_controllers_msgs::SetTasks::Request & req, hqp_controllers_msgs::SetTasks::Response &res);
    //bool removeTasks(hqp_controllers_msgs::RemoveTasks::Request & req, hqp_controllers_msgs::RemoveTasks::Response &res);
    //bool visualizeTaskLinks(hqp_controllers_msgs::VisualizeTaskLinks::Request & req, hqp_controllers_msgs::VisualizeTaskObjects::Response &res);
};


} //end namespace hqp_controllers

#endif
