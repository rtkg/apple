#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <hqp_controllers/task.h>

namespace hqp_controllers {

class TaskManager
{
public:
    TaskManager();
 bool isInitialized();
 void initialize(boost::shared_ptr<KDL::Tree> kinematics);
 void addTask(boost::shared_ptr<Task>);
 void removeTask(unsigned int id);

private:
 bool initialized_;
 boost::shared_ptr<KDL::Tree> kinematics_;
 boost::shared_ptr<std::map<boost::shared_ptr<Task>, unsigned int> > tasks_; ///< Map identifying the held tasks by their id's

};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
