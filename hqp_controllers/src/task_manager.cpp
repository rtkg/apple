#include <hqp_controllers/task_manager.h>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager() : hqp_computed_(false)
{
    tasks_.reset(new std::map<unsigned int, boost::shared_ptr<Task> >);
    t_objs_.reset(new std::map<unsigned int, boost::shared_ptr<TaskObject> >);
    hqp_.reset(new std::map<unsigned int, boost::shared_ptr<HQPStage> >);
}
//----------------------------------------------
TaskManager::TaskManager(boost::shared_ptr<KDL::Tree> k_tree) : k_tree_(k_tree), hqp_computed_(false)
{
    tasks_.reset(new std::map<unsigned int, boost::shared_ptr<Task> >);
    t_objs_.reset(new std::map<unsigned int, boost::shared_ptr<TaskObject> >);
    hqp_.reset(new std::map<unsigned int, boost::shared_ptr<HQPStage> >);
}
//----------------------------------------------
void TaskManager::setKinematicTree(boost::shared_ptr<KDL::Tree> k_tree) {k_tree_ = k_tree;}
//----------------------------------------------
bool TaskManager::addTaskObject(boost::shared_ptr<TaskObject> t_obj)
{
    //Make sure an object with the same id doesn't already exist in the map
    std::pair<std::map<unsigned int, boost::shared_ptr<TaskObject> >::iterator,bool> it;
    it = t_objs_->insert(std::pair<unsigned int, boost::shared_ptr<TaskObject> >(t_obj->getId(),t_obj));
    if(it.second == false)
    {
        ROS_ERROR("Cannot add task object with id %d since it already exists.", t_obj->getId());
        return false;
    }

    //=================== DEBUG PRINT =========================
    //std::cout<<"ADDED TASK OBJECT: "<<std::endl<< *(t_obj);
    //=================== DEBUG PRINT END =========================

    return true;
}
//----------------------------------------------
boost::shared_ptr<KDL::Tree> TaskManager::getKinematicTree()const {return k_tree_;}
//----------------------------------------------
unsigned int TaskManager::getValidTaskObjectId() const
{
    if(t_objs_->size() == 0)
        return 0;
    else
        return t_objs_->rbegin()->first + 1;//The object with the largest id (id's are also map keys) is at the end
}
//----------------------------------------------
unsigned int TaskManager::getValidTaskId() const
{
    if(tasks_->size() == 0)
        return 0;
    else
        return tasks_->rbegin()->first + 1;    //The task with the largest id (id's are also map keys) is at the end
}
//----------------------------------------------
bool TaskManager::addTask(boost::shared_ptr<Task> task)
{
    //make sure the task objects associated with the given task exist
    TaskObject t_obj;
    for(unsigned int i = 0; i<task->getTaskObjects()->size(); i++)
        if(!getTaskObject(task->getTaskObjects()->at(i).getId(), t_obj))
        {
            ROS_ERROR("Cannot add task with id %d since the required task object with id %d does not exist in the task objects map.", task->getId(),task->getTaskObjects()->at(i).getId());
            return false;
        }

    //Make sure a task with the same id doesn't already exist in the map
    std::pair<std::map<unsigned int, boost::shared_ptr<Task> >::iterator,bool> it;
    it = tasks_->insert(std::pair<unsigned int, boost::shared_ptr<Task> >(task->getId(),task));
    if(it.second == false)
    {
        ROS_ERROR("Cannot add task with id %d since it already exists.", task->getId());
        return false;
    }

    //=================== DEBUG PRINT =========================
    //std::cout<<"ADDED TASK: "<<std::endl<< *(task);
    //=================== DEBUG PRINT END =========================

    return true;
}
//----------------------------------------------
bool TaskManager::removeTaskObject(unsigned int id)
{
    //check whether an object with that id exists and remove it in case
    std::map<unsigned int,boost::shared_ptr<TaskObject> >::iterator it = t_objs_->find(id);
     if(it == t_objs_->end())
     {
         ROS_WARN("TaskManager::removeTaskObject(...): cannot remove task object with id %d because it does not exist.",id);
         return false;
     }
     else
          t_objs_->erase(it);

   return true;
}
//----------------------------------------------
bool TaskManager::removeTask(unsigned int id)
{
    //check whether a task with that id exists and remove it in case
    std::map<unsigned int,boost::shared_ptr<Task> >::iterator it = tasks_->find(id);
     if(it == tasks_->end())
     {
         ROS_WARN("TaskManager::removeTask(...): cannot remove task with id %d because it does not exist.",id);
         return false;
     }
     else
          tasks_->erase(it);

   return true;
}
//----------------------------------------------
boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<TaskObject> > > TaskManager::getTaskObjects()const{return t_objs_;}
//----------------------------------------------
void TaskManager::computeTaskObjectsKinematics()
{
    //iterate through all task objects and compute the kinematics
    for (std::map<unsigned int, boost::shared_ptr<TaskObject> >::iterator it=t_objs_->begin(); it!=t_objs_->end(); ++it)
        it->second->computeKinematics();
}
//----------------------------------------------
bool TaskManager::getDQ(Eigen::VectorXd& dq)const
{
    if(!hqp_computed_)
        return false;

    //final solution is in the last stage of the hqp
    ROS_ASSERT(hqp_->rbegin()->second->solved_); //just to be sure ...
    dq = *(hqp_->rbegin()->second->x_);
    return true;
}
//----------------------------------------------
bool TaskManager::getTaskObject(unsigned int id, TaskObject& t_obj)const
{
    std::map<unsigned int,boost::shared_ptr<TaskObject> >::iterator it = t_objs_->find(id);
     if(it == t_objs_->end())
     {
         ROS_WARN("TaskManager::getTaskObject(...): could not find task object with id %d.",id);
         return false;
     }
     else
         t_obj = *it->second;

    return true;
}
//----------------------------------------------
void TaskManager::getTaskStatuses(hqp_controllers_msgs::TaskStatuses& t_statuses)
{
    t_statuses.statuses.clear();

    for (std::map<unsigned int, boost::shared_ptr<Task> >::iterator it=tasks_->begin(); it!=tasks_->end(); ++it)
    {
        hqp_controllers_msgs::TaskStatus status;
        status.id = it->second->getId();
        unsigned int t_dim = it->second->getTaskFunction()->size();
        ROS_ASSERT(t_dim == it->second->getTaskVelocity()->size()); //just to be sure ...

        for(unsigned int i=0; i < t_dim; i++)
        {
            status.e.push_back( (*it->second->getTaskFunction())(i));
            status.de.push_back( (*it->second->getTaskVelocity())(i));
        }

        status.sse = it->second->getSSE();

        t_statuses.statuses.push_back(status);
    }
}
//----------------------------------------------
bool TaskManager::getTaskGeometryMarkers(visualization_msgs::MarkerArray& t_geoms,Eigen::VectorXi const& vis_ids)const
{
    for(unsigned int i=0; i<vis_ids.size(); i++)
    {
        //Make sure the task object with id vis_ids(i) exists
        std::map<unsigned int,boost::shared_ptr<TaskObject> >::iterator it = t_objs_->find(vis_ids(i));
        if(it == t_objs_->end())
        {
            ROS_ERROR("Could not find task object with id %d. Associated geometries can not be visualized.", vis_ids(i));
            return false;
        }

        //Add markers for all geometries associated to the object with id vis_ids(i)
        for(unsigned int j=0; j<it->second->getGeometries()->size();j++)
            it->second->getGeometries()->at(j)->addMarker(t_geoms);

    }
    return true;
}

//----------------------------------------------
void TaskManager::computeHQP()
{
    hqp_->clear();
    hqp_computed_ = false;
    //iterate through all tasks, compute task velocities and jacobians and insert the corresponding HQP stages
    for (std::map<unsigned int, boost::shared_ptr<Task> >::iterator task_it=tasks_->begin(); task_it!=tasks_->end(); ++task_it)
    {
        task_it->second->computeTask();
        unsigned int priority =  task_it->second->getPriority();

        //if no stage with the given priority is in the hqp yet, create one, otherwise append the task to the existing stage
        std::map<unsigned int,boost::shared_ptr<HQPStage> >::iterator stage_it = hqp_->find(priority);
        if(stage_it == hqp_->end())
            (*hqp_)[priority] = boost::shared_ptr<HQPStage>(new HQPStage(*task_it->second));
        else
            stage_it->second->appendTask(*task_it->second);
    }

    if(hqp_solver_.solve(*hqp_))
        hqp_computed_ = true;

}
//----------------------------------------------
void TaskManager::writeHQP()
{
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
}
//----------------------------------------------
}//end namespace hqp_controllers
