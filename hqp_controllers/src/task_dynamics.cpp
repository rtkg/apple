#include <hqp_controllers/task_dynamics.h>
#include <ros/ros.h>

namespace hqp_controllers{

//----------------------------------------------------------
TaskDynamics::TaskDynamics() : dim_(0){};
//----------------------------------------------------------
TaskDynamics::TaskDynamics(unsigned int dim) : dim_(dim){};
//----------------------------------------------------------
unsigned int TaskDynamics::getDimension()const{return dim_;}
//----------------------------------------------------------
LinearTaskDynamics::LinearTaskDynamics(unsigned int dim) : TaskDynamics(dim){};
//----------------------------------------------------------
void LinearTaskDynamics::computeDX()const
{
    ROS_INFO("TODO ...");
}
//----------------------------------------------------------
}//end namespace hqp_controllers
