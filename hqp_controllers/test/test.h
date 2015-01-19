#ifndef TEST_H
#define TEST_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <hqp_controllers_msgs/SetTaskObject.h>

namespace hqp_controllers
{
class HQPTest
{
public:

    HQPTest();

private:

    ros::NodeHandle nh_;

    //  ros::ServiceServer get_icr_srv_;
    //  ros::ServiceServer compute_icr_srv_;
    //  ros::ServiceServer compute_sz_srv_;
    ros::ServiceServer set_task_obj_srv_;
    //  ros::ServiceServer toggle_mode_srv_;
    //  ros::ServiceServer set_qs_srv_;
    //  ros::ServiceServer set_active_phl_srv_;
    //  ros::ServiceServer set_phl_param_srv_;
    //  ros::ServiceServer save_icr_srv_;
    //  ros::Subscriber ct_pts_sub_;
    //  ros::Publisher icr_cloud_pub_;
    //  ros::Publisher icr_pub_;

    boost::mutex lock_;

    /////////////////
    //  CALLBACKS  //
    /////////////////

    bool setTaskObject(hqp_controllers_msgs::SetTaskObject::Request & req, hqp_controllers_msgs::SetTaskObject::Response &res);
};

}//end namespace hqp controllers

#endif
