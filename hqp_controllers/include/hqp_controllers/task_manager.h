#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <hqp_controllers/task_link.h>
#include <hqp_controllers/task.h>
#include <hqp_controllers/hqp.h>
#include <hqp_controllers_msgs/TaskStatuses.h>
#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <map>
#include <hardware_interface/joint_command_interface.h>
#include <visualization_msgs/MarkerArray.h>

namespace hqp_controllers {

class TaskManager
{
public:
    TaskManager();

    bool addTask(boost::shared_ptr<Task> task);

    void reset();
  //  bool removeTask(unsigned int id);

    /**Computes the task jacobians and velocities of all tasks*/
    void updateTasks();
   /**solves the corresponding HQP. The solution is returned in dq */
    void computeHQP();

    //void writeHQP();

    unsigned int getValidTaskId() const;
    bool getTask(unsigned int id, boost::shared_ptr<Task>& task)const;
    void getTaskStatuses(hqp_controllers_msgs::TaskStatuses& t_statuses);
    bool getTaskGeometryMarkers(visualization_msgs::MarkerArray& markers, Eigen::VectorXi const& vis_ids)const;

    bool getDQ(Eigen::VectorXd& dq)const;

private:
    //boost::shared_ptr<KDL::Tree> k_tree_;
    std::map<unsigned int, boost::shared_ptr<Task> > tasks_;
    std::map<unsigned int, boost::shared_ptr<HQPStage> > hqp_;
    bool hqp_computed_;

    HQPSolver hqp_solver_;
};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
