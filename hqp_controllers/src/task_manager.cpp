#include <hqp_controllers/task_manager.h>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager()
{
    tasks_.reset(new std::map<unsigned int, boost::shared_ptr<Task> >);
    t_objs_.reset(new std::map<unsigned int, boost::shared_ptr<TaskObject> >);
    hqp_.reset(new std::map<unsigned int, boost::shared_ptr<HQPStage> >);
}
//----------------------------------------------
TaskManager::TaskManager(boost::shared_ptr<KDL::Tree> k_tree) : k_tree_(k_tree)
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
    //make sure both task objects associated with the given task exist
    if(getTaskObject(task->getTaskObjects().first->getId()).get() == NULL)
    {
        ROS_ERROR("Cannot add task with id %d since the required task object with id %d does not exist in the task objects map.", task->getId(),task->getTaskObjects().first->getId());
        return false;
    }
    else if(getTaskObject(task->getTaskObjects().second->getId()).get() == NULL)
    {
        ROS_ERROR("Cannot add task with id %d since the required task object with id %d does not exist in the task objects map.", task->getId(),task->getTaskObjects().second->getId());
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
    //  std::cout<<"ADDED TASK: "<<std::endl<< *(task);
    //=================== DEBUG PRINT END =========================

    return true;
}
//----------------------------------------------
void TaskManager::removeTask(unsigned int id)
{
    ROS_WARN("TaskManager::addTask(...) is not implemented yet!");
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
boost::shared_ptr<TaskObject> TaskManager::getTaskObject(unsigned int id)const
{
    boost::shared_ptr<TaskObject> t_obj; //gets initialized to NULL

    std::map<unsigned int,boost::shared_ptr<TaskObject> >::iterator it = t_objs_->find(id);
    if(it == t_objs_->end())
        ROS_WARN("TaskManager::getTaskObject(...): could not find task object with id %d.",id);
    else
        t_obj = it->second;

    return t_obj;
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

    if(!hqp_solver_.solve(*hqp_))
        ROS_BREAK();
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
