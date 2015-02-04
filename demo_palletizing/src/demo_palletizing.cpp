#include <demo_palletizing/demo_palletizing.h>

namespace demo_palletizing
{
//-----------------------------------------------------------------
DemoPalletizing::DemoPalletizing()
{

    //handle to home
    nh_ = ros::NodeHandle("~");
    //global handle
    n_ = ros::NodeHandle();

    //get params
    nh_.param<std::string>("set_task_service", set_task_name,"/set_task");
    nh_.param<std::string>("task_state_topic", task_state_topic,"/task_state");

    //initialize variables
    task_status_changed = false;
    task_success = false;

    //register callbacks
    start_demo_srv = nh_.advertiseService("start_demo", &DemoPalletizing::start_demo, this);;
    taks_status_sub = n_.subscribe(task_state_topic, 10, &DemoPalletizing::stateCallback, this);
    send_arm_task = n_.serviceClient<hqp_controllers_msgs::SetTasks>(set_task_name);

}
//-----------------------------------------------------------------
void DemoPalletizing::stateCallback( const hqp_controllers_msgs::TaskStatusesPtr& msg) {

    //inside this block it is safe to modify task status and success
    boost::mutex::scoped_lock lock(arm_tasks_m);
    //here monitor specific task and once status changes do:
    task_status_changed = true;
    task_success = true;
    cond.notify_one();
}

bool DemoPalletizing::start_demo(std_srvs::Empty::Request  &req,
	std_srvs::Empty::Response &res ) {

    //call some blocking service for moving the forklift to position 
    
    //add a task to the arm controller
    {
	ROS_INFO("entering the service lock");
	//inside this scope {} it is safe to access shared data like task_status, task_success
	boost::mutex::scoped_lock lock (arm_tasks_m);
	/*
	hqp_controllers_msgs::SetTasks task;
	if(!send_arm_task.call(task)) {
	    ROS_ERROR("could not send task to arm");
	    //handle errors or
	    ros::shutdown();
	    return false;
	}
	*/
	while(!task_status_changed) {
	    cond.wait(lock);
	}
	if(!task_success) {
	    //handle errors, recover or
	    ros::shutdown();
	    return false;
	}
	ROS_INFO("made it through the lock");
    }
    //here lock is released, so task status cannot be checked safely.
    //you can add another block for the next task...

    return true;
}
}//end namespace demo_palletizing


/////////////////////////////////
//           MAIN              //
/////////////////////////////////


//---------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_palletizing");

    demo_palletizing::DemoPalletizing demo;

    ROS_INFO("Demo palletizing node ready");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
//---------------------------------------------------------------------
