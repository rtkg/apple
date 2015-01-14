#include <hqp_controllers/task_manager.h>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager() : initialized_(false){}
//----------------------------------------------
bool TaskManager::isInitialized() {return initialized_;}
//----------------------------------------------
void TaskManager::initialize(boost::shared_ptr<KDL::Tree> kinematics)
{
    kinematics_ = kinematics;

    initialized_ = true;
}

//----------------------------------------------
}//end namespace hqp_controllers
