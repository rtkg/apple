#include <hqp_controllers/task_manager.h>
#include <kdl/jntarray.hpp>

namespace hqp_controllers{

//----------------------------------------------
TaskManager::TaskManager() : initialized_(false){}
//----------------------------------------------
bool TaskManager::isInitialized() const {return initialized_;}
//----------------------------------------------
void TaskManager::initialize (boost::shared_ptr<KDL::Tree> k_tree, boost::shared_ptr<std::vector<boost::shared_ptr<TaskObject> > > t_obj_list)
{
    k_tree_ = k_tree;
    t_obj_list_ = t_obj_list;

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

    initialized_ = true;
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
    for(int i=0; i<t_obj_list_->size(); i++)
        t_obj_list_->at(i)->computeKinematics();
}
//----------------------------------------------
}//end namespace hqp_controllers
