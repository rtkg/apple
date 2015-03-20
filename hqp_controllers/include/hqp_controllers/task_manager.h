#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <hqp_controllers/task_link.h>
#include <hqp_controllers/task.h>
#include <hqp_controllers/hqp.h>
#include <hqp_controllers_msgs/TaskStatusArray.h>
#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <map>
#include <hardware_interface/joint_command_interface.h>
#include <visualization_msgs/MarkerArray.h>
#include <time.h>

namespace hqp_controllers {
//--------------------------------------------------------
class Timer
{
public:
    Timer();

    void iterate();
   // void reset();
    double getCTime()const;

private:
    struct timeval t_;//, t_prev_;
    double c_time_;
};
//--------------------------------------------------------
class TaskManager
{
public:
    TaskManager();

    bool addTask(boost::shared_ptr<Task> task);

    void reset();
    bool removeTask(unsigned int id);

    /**Computes the task jacobians and velocities of all tasks and solves the HQP*/
    void updateTasks();

    unsigned int getValidTaskId() const;
    bool getTask(unsigned int id, boost::shared_ptr<Task> task)const;
    void getTaskStatusArray(hqp_controllers_msgs::TaskStatusArray& t_status_array);
    bool getTaskGeometryMarkers(visualization_msgs::MarkerArray& markers, Eigen::VectorXi const& vis_ids)const;

    bool getDQ(Eigen::VectorXd& dq)const;

private:
    //boost::shared_ptr<KDL::Tree> k_tree_;
    std::map<unsigned int, boost::shared_ptr<Task> > tasks_;
    std::map<unsigned int, boost::shared_ptr<HQPStage> > hqp_;
    bool hqp_computed_;
    Timer timer_;

    HQPSolver hqp_solver_;
};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
