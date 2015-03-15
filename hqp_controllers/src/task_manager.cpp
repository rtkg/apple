#include <hqp_controllers/task_manager.h>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager() : hqp_computed_(false){}
//----------------------------------------------
unsigned int TaskManager::getValidTaskId() const
{
   if(tasks_.size() == 0)
        return 0;
    else
        return tasks_.rbegin()->first + 1;  //The task with the largest id (id's are also map keys) is at the end
}
//----------------------------------------------
bool TaskManager::addTask(boost::shared_ptr<Task> task)
{
    //Make sure a task with the same id doesn't already exist in the map
    std::pair<std::map<unsigned int, boost::shared_ptr<Task> >::iterator,bool> it;
    it = tasks_.insert(std::pair<unsigned int, boost::shared_ptr<Task> >(task->getId(),task));
    if(it.second == false)
    {
        ROS_ERROR("Cannot add task with id %d since it already exists.", task->getId());
        return false;
    }

    //=================== DEBUG PRINT =============================
    //std::cout<<"ADDED TASK: "<<std::endl<< *(task);
    //=================== DEBUG PRINT END =========================

    return true;
}
//----------------------------------------------
bool TaskManager::removeTask(unsigned int id)
{
    //check whether a task with that id exists and remove it in case
    std::map<unsigned int,boost::shared_ptr<Task> >::iterator it = tasks_.find(id);
    if(it == tasks_.end())
    {
        ROS_WARN("TaskManager::removeTask(...): cannot remove task with id %d because it does not exist.",id);
        return false;
    }
    else
        tasks_.erase(it);

//    hqp_.clear();
//    hqp_computed_ = false;

    return true;
}
//----------------------------------------------
//boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<TaskObject> > > TaskManager::getTaskObjects()const{return t_objs_;}
//----------------------------------------------
void TaskManager::updateTasks()
{
    //iterate through all tasks and compute the kinematics, task functions & task jacobians
    for (std::map<unsigned int, boost::shared_ptr<Task> >::iterator it=tasks_.begin(); it!=tasks_.end(); ++it)
    {
   //     std::cerr<<"trying to update task with id: "<<it->second->getId()<<", corresponding map key: "<<it->first<<std::endl;
   //     std::cerr<<"task boost ptr: "<<it->second<<", task ptr: "<<it->second.get()<<std::endl;
        it->second->updateTask();
    }
}
//----------------------------------------------
bool TaskManager::getDQ(Eigen::VectorXd& dq)const
{
    if(!hqp_computed_)
        return false;

    //final solution is in the last stage of the hqp
    ROS_ASSERT(hqp_.rbegin()->second->solved_); //just to be sure ...
    dq = hqp_.rbegin()->second->x_;
    return true;
}
//----------------------------------------------
bool TaskManager::getTask(unsigned int id, boost::shared_ptr<Task> task)const
{
    std::map<unsigned int,boost::shared_ptr<Task> >::const_iterator it = tasks_.find(id);
    if(it == tasks_.end())
    {
        ROS_WARN("TaskManager::getTask(...): could not find task with id %d.",id);
        return false;
    }
    else
        task = it->second;

    return true;
}
//----------------------------------------------
void TaskManager::reset()
{
   tasks_.clear();
   hqp_.clear();
   hqp_computed_ = false;
}
//----------------------------------------------
void TaskManager::getTaskStatusArray(hqp_controllers_msgs::TaskStatusArray& t_status_array)
{
    t_status_array.statuses.clear();

    for (std::map<unsigned int, boost::shared_ptr<Task> >::iterator it=tasks_.begin(); it!=tasks_.end(); ++it)
    {
        hqp_controllers_msgs::TaskStatus status;
        status.id = it->second->getId();
        for(unsigned int i=0; i < it->second->getTaskFunction().rows(); i++)
        {
            status.e.push_back(it->second->getTaskFunction()(i));
            status.de.push_back(it->second->getTaskVelocity()(i));
        }

        status.progress = it->second->getTaskProgress();

        t_status_array.statuses.push_back(status);
    }
}
//----------------------------------------------
bool TaskManager::getTaskGeometryMarkers(visualization_msgs::MarkerArray& markers, Eigen::VectorXi const& vis_ids)const
{
    for(unsigned int i=0; i<vis_ids.size(); i++)
    {
        //Make sure the task with id vis_ids(i) exists
        std::map<unsigned int, boost::shared_ptr<Task> >::const_iterator it = tasks_.find(vis_ids(i));
        if(it == tasks_.end())
        {
            ROS_ERROR("Could not find task with id %d. Associated geometries can not be visualized.", vis_ids(i));
            return false;
        }

        //Add markers for all geometries associated to the task with id vis_ids(i)
        for(unsigned int i=0; i < it->second->getTaskLinks().size(); i++)
        {
            TaskLink* link = it->second->getTaskLinks().at(i).get();
            for(unsigned int j=0; j < link->getGeometries().size(); j++)
                link->getGeometries().at(j)->addMarker(markers);
        }
    }
    return true;
}
//----------------------------------------------
void TaskManager::computeHQP()
{
   hqp_.clear();
    for (std::map<unsigned int, boost::shared_ptr<Task> >::iterator task_it=tasks_.begin(); task_it!=tasks_.end(); ++task_it)
    {

        unsigned int priority =  task_it->second->getPriority();

       //if no stage with the given priority is in the hqp yet, create one, otherwise append the task to the existing stage
        std::map<unsigned int,boost::shared_ptr<HQPStage> >::const_iterator stage_it = hqp_.find(priority);
        if(stage_it == hqp_.end())
            hqp_[priority] = boost::shared_ptr<HQPStage>(new HQPStage(*task_it->second));
        else
            stage_it->second->appendTask(*task_it->second);
    }

    if(hqp_solver_.solve(hqp_))
        hqp_computed_ = true;
    else
        hqp_computed_ = false;
}
//----------------------------------------------
//void TaskManager::writeHQP()
//{
    //    FILE* f=fopen ("/home/rkg/Desktop/hqp.dat","w");
    //    if(f==NULL)
    //    {
    //        ROS_ERROR("Error in TaskManager::writeHQP(): could not open hqp.dat");
    //        ROS_BREAK();
    //    }

    //    for (std::map<unsigned int, boost::shared_ptr<HQPStage> >::iterator it=hqp_->begin(); it!=hqp_->end(); ++it)
    //    {
    //        HQPStage stage = (*it->second);
    //        std::ostringstream str;
    //        str<< (*stage.de_);
    //        fprintf(f, "%s, \n", str.str().c_str());
    //    }
    //    fclose (f);
//}
//----------------------------------------------
}//end namespace hqp_controllers
