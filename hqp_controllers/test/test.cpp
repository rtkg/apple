#include "test.h"



namespace hqp_controllers
{
//-----------------------------------------------------------------
HQPTest::HQPTest()
{
    set_task_obj_srv_ = nh_.advertiseService("set_task_object",&HQPTest::setTaskObject,this);
}
//-----------------------------------------------------------------
bool HQPTest::setTaskObject(hqp_controllers_msgs::SetTaskObject::Request & req, hqp_controllers_msgs::SetTaskObject::Response &res)
{
    lock_.lock();
    ROS_INFO("Called setTaskObject callback...");
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
