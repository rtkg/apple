#ifndef DEMO_PALLETIZING_H
#define DEMO_PALLETIZING_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <std_srvs/Empty.h>
#include <hqp_controllers_msgs/TaskStatuses.h>
#include <hqp_controllers_msgs/SetTasks.h>

namespace demo_palletizing
{
class DemoPalletizing
{
public:

    DemoPalletizing();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle n_;
    boost::mutex arm_tasks_m;
    boost::condition_variable cond;

    bool task_status_changed;
    bool task_success;

    ros::Subscriber taks_status_sub;
    ros::ServiceClient send_arm_task;
    ros::ServiceServer start_demo_srv;

    std::string set_task_name, task_state_topic;
    /////////////////
    //  CALLBACKS  //
    /////////////////
    void stateCallback( const hqp_controllers_msgs::TaskStatusesPtr& msg);
    bool start_demo(std_srvs::Empty::Request  &req,
	    std_srvs::Empty::Response &res );

};

}//end namespace hqp controllers

#endif



