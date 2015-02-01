#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <hqp_controllers/task_object.h>
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
    TaskManager(boost::shared_ptr<KDL::Tree> k_tree);

    void setKinematicTree(boost::shared_ptr<KDL::Tree> k_tree);
    bool addTaskObject(boost::shared_ptr<TaskObject> t_obj);
    bool addTask(boost::shared_ptr<Task> task);

    void removeTask(unsigned int id);
    void computeTaskObjectsKinematics();
    /**Computes the task jacobians and velocities of all tasks ans solves the corresponding HQP. The solution is returned in dq */
    void computeHQP();

    void writeHQP();

    boost::shared_ptr<KDL::Tree> getKinematicTree()const;
    unsigned int getValidTaskId() const;
    unsigned int getValidTaskObjectId() const;
    bool getTaskObject(unsigned int id, TaskObject& t_obj)const;
    void getTaskStatuses(hqp_controllers_msgs::TaskStatuses& t_statuses);
    bool getTaskGeometryMarkers(visualization_msgs::MarkerArray& t_geoms,Eigen::VectorXi const& vis_ids)const;
    boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<TaskObject> > > getTaskObjects()const;
    bool getDQ(Eigen::VectorXd& dq)const;

private:
    boost::shared_ptr<KDL::Tree> k_tree_;
    boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<TaskObject> > > t_objs_;
    boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<Task> > > tasks_;
    boost::shared_ptr<std::map<unsigned int, boost::shared_ptr<HQPStage> > > hqp_;
    bool hqp_computed_;

    HQPSolver hqp_solver_;
};

}//end namespace hqp_controllers

#endif // TASK_MANAGER_H
