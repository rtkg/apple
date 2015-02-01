#include "test.h"



namespace hqp_controllers
{
//-----------------------------------------------------------------
HQPTest::HQPTest()
{
    set_task_obj_srv_ = nh_.advertiseService("set_task_objects",&HQPTest::setTaskObjects,this);
}
//-----------------------------------------------------------------
bool HQPTest::setTaskObjects(hqp_controllers_msgs::SetTaskObjects::Request & req, hqp_controllers_msgs::SetTaskObjects::Response &res)
{
    lock_.lock();
    ROS_INFO("Called setTaskObjects callback...");
    lock_.unlock();
    return res.success;
}
//-----------------------------------------------------------------
}//end namespace hqp_controllers


/////////////////////////////////
//           MAIN              //
/////////////////////////////////


//---------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "icr_server");

    hqp_controllers::HQPTest hqp_test;
    ROS_INFO("HQP test node ready");
    while(ros::ok())
    {
        //do smthing
        ros::spinOnce();
    }
    return 0;
}
//---------------------------------------------------------------------
