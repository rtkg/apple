#include <hqp_controllers/task_manager.h>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager()
{
    t_objs_.reset(new std::map<unsigned int, boost::shared_ptr<TaskObject> >);
}
//----------------------------------------------
TaskManager::TaskManager(boost::shared_ptr<KDL::Tree> k_tree) : k_tree_(k_tree)
{
    t_objs_.reset(new std::map<unsigned int, boost::shared_ptr<TaskObject> >);
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
    //The object with the largest id (id's are also map keys) is at the end
    return t_objs_->rbegin()->first + 1;
}
//----------------------------------------------
void TaskManager::addTask(boost::shared_ptr<Task>)
{

}
//----------------------------------------------
void TaskManager::removeTask(unsigned int id)
{

}
//------------------t----------------------------
void TaskManager::computeTaskObjectsKinematics()
{
    //iterate through all task objects and compute the kinematics
    for (std::map<unsigned int, boost::shared_ptr<TaskObject> >::iterator it=t_objs_->begin(); it!=t_objs_->end(); ++it)
        it->second->computeKinematics();
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
}//end namespace hqp_controllers
