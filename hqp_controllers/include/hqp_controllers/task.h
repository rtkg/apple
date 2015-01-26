#ifndef TASK_H
#define TASK_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <hqp_controllers/task_object.h>
#include <hqp_controllers/task_dynamics.h>

namespace hqp_controllers {
//----------------------------------------------------------------
enum TaskType {UNDEFINED_TASK = 0, POINT_IN_HALFSPACE = 1};
//----------------------------------------------------------------
class Task
{
public:
    Task(unsigned int id, unsigned int priority, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    void setId(unsigned int id);
    void setPriority(unsigned int priority);
    void setSign(std::string const& sign);
    //  void setTaskDynamics(boost::shared_ptr<std::vector<TaskDynamics> > t_dynamics);

    boost::shared_ptr<TaskDynamics> getTaskDynamics()const;
    unsigned int getId();
    TaskType getType();
    unsigned int getPriority();
    std::string getSign()const;
    boost::shared_ptr<Eigen::MatrixXd> getTaskJacobian()const;
    void getTaskFunction(Eigen::VectorXd& e)const;
    //    boost::shared_ptr<Eigen::VectorXd> getTaskVelocity()const;
    std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > getTaskObjects()const;

    static boost::shared_ptr<Task>  makeTask(unsigned int id, unsigned int priority, TaskType type, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics); ///<factory method
    //** Needs to be checked whether the task objects have appropriate properties in each derived task class */
    //  virtual void setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs) = 0;

    //**Computes task function, velocity and jacobians. Assumes that the kinematics of the corresponding task objects have been computed prior to the call to this function.*/
    virtual void computeTask()=0;

protected:
    Task(){};

    TaskType type_;
    unsigned int id_;
    unsigned int priority_;
    std::string sign_;
    unsigned int dim_;
    boost::shared_ptr<TaskDynamics> t_dynamics_;
    std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs_;

    boost::shared_ptr<Eigen::MatrixXd> A_; ///< task jacobian
    boost::shared_ptr<Eigen::MatrixXd> E_; ///< task function state matrix, rows \in Task::dim_ correspond to the task function dynamics of the single task dimensions, columns \in Task::t_dynamics_->getDimension()+1 hold the corresponding derivatives
    ros::Time t_prev_;

    void updateTaskFunctionMatrix();
};
//----------------------------------------------------------------
class PointInHalfspace: public Task
{
public:

    PointInHalfspace(unsigned int id, unsigned int priority, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    //  virtual void setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs);
    virtual void computeTask();

protected:

    PointInHalfspace(){};

private:

    //**Helper function to make sure that the given task objects are valid in the context of the task */
    void verifyTaskObjects();
};
//----------------------------------------------------------------
} //end namespace hqp_controllers

#endif // TASK_H
