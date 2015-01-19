#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <hqp_controllers/task_object.h>
#include <hqp_controllers/task.h>
#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <map>
#include <hardware_interface/joint_command_interface.h>

namespace hqp_controllers {

class TaskManager
{
public:

    TaskManager();
    TaskManager(boost::shared_ptr<KDL::Tree> k_tree);

    void setKinematicTree(boost::shared_ptr<KDL::Tree> k_tree);
    bool addTaskObject(boost::shared_ptr<TaskObject> t_obj);
    void addTask(boost::shared_ptr<Task>);
    void removeTask(unsigned int id);
    void computeTaskObjectsKinematics();

    boost::shared_ptr<KDL::Tree> getKinematicTree()const;
    unsigned int getValidTaskObjectId() const;
private:

    boost::shared_ptr<KDL::Tree> k_tree_;
    boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<TaskObject> > > t_objs_;
};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
