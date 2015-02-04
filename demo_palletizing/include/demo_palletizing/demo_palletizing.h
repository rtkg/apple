#ifndef DEMO_PALLETIZING_H
#define DEMO_PALLETIZING_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace demo_palletizing
{
class DemoPalletizing
{
public:

    DemoPalletizing();

private:

    ros::NodeHandle nh_;


    /////////////////
    //  CALLBACKS  //
    /////////////////

    // bool setTaskObjects(hqp_controllers_msgs::SetTaskObjects::Request & req, hqp_controllers_msgs::SetTaskObjects::Response &res);
};

}//end namespace hqp controllers

#endif



