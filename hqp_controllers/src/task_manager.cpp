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
//=================== DEBUG PRINTING ==============================
//    std::cout<<"TASK OBJECT LIST:"<<std::endl;
//    for(int i=0; i<t_obj_list_->size(); i++)
//    {
//        std::cout<<"frame: "<<t_obj_list_->at(i)->getFrame()<<std::endl;
//        for(int j=0; j<t_obj_list_->at(i)->getGeometries()->size();j++)
//        {
//            boost::shared_ptr<TaskGeometry> t_geom=t_obj_list_->at(i)->getGeometries()->at(j);
//            std::cout<<"geom frame: "<<t_geom->getFrame()<<std::endl;
//            std::cout<<"geom type: "<<t_geom->getType()<<std::endl;
//            if(t_geom->getType()==POINT)
//            {
//                std::cout<<"POINT:"<<std::endl;
//                boost::shared_ptr<Eigen::Vector3d> p = static_cast<Point*>(t_geom.get())->getPosition();
//                std::cout<<"p: "<<p->transpose()<<std::endl;
//            }
//            if(t_geom->getType()==PLANE)
//            {
//                std::cout<<"PLANE:"<<std::endl;
//                boost::shared_ptr<Eigen::Vector3d> n = static_cast<Plane*>(t_geom.get())->getNormal();
//                double d = static_cast<Plane*>(t_geom.get())->getOffset();
//                std::cout<<"n: "<<n->transpose()<<" d: "<<d<<std::endl;
//            }
//            else if(t_geom->getType() == CAPSULE)
//            {
//                std::cout<<"CAPSULE:"<<std::endl;
//                boost::shared_ptr<Eigen::Vector3d> p = static_cast<Capsule*>(t_geom.get())->getStartPosition();
//                boost::shared_ptr<Eigen::Vector3d> t = static_cast<Capsule*>(t_geom.get())->getEndPosition();
//                double r = static_cast<Capsule*>(t_geom.get())->getRadius();
//                std::cout<<"p: "<<p->transpose()<<std::endl;
//                std::cout<<"t: "<<t->transpose()<<std::endl;
//                std::cout<<"r: "<<r<<std::endl;
//            }
//            std::cout<<std::endl<<std::endl;
//        }
//    }
//=================== END DEBUG PRINTING ==============================

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
}//end namespace hqp_controllers
