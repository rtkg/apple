#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <hqp_controllers/task_object.h>
#include <hqp_controllers/task.h>
#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <vector>
#include <hardware_interface/joint_command_interface.h>

namespace hqp_controllers {

class TaskManager
{
public:
    TaskManager();
 bool isInitialized() const;
 void initialize (boost::shared_ptr<KDL::Tree> k_tree, boost::shared_ptr<std::vector<boost::shared_ptr<TaskObject> > > t_obj_list);
 void addTask(boost::shared_ptr<Task>);
 void removeTask(unsigned int id);
 void computeTaskObjectsKinematics();

private:
 bool initialized_;
 boost::shared_ptr<KDL::Tree> k_tree_;
 boost::shared_ptr<std::vector<boost::shared_ptr<TaskObject> > > t_obj_list_; ///< list of all task objects - needs to be updated every time a task is added/removed

};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
