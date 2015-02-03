#ifndef TASK_H
#define TASK_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <hqp_controllers/task_object.h>
#include <hqp_controllers/task_dynamics.h>

namespace hqp_controllers {
//----------------------------------------------------------------
enum TaskType {UNDEFINED_TASK = 0, PROJECT_POINT_PLANE = 1, JOINT_SETPOINT = 2, JOINT_VELOCITY_LIMITS = 3, PARALLEL_LINES = 4, ANGLE_LINES = 5};
//----------------------------------------------------------------
class Task
{
public:
    Task(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    void setId(unsigned int id);
    void setPriority(unsigned int priority);
    void setSign(std::string const& sign);
    //  void setTaskDynamics(boost::shared_ptr<std::vector<TaskDynamics> > t_dynamics);

    boost::shared_ptr<TaskDynamics> getTaskDynamics()const;
    unsigned int getId()const;
    TaskType getType()const;
    unsigned int getDimension()const;
    unsigned int getPriority()const;
    std::string getSign()const;
    boost::shared_ptr<Eigen::MatrixXd> getTaskJacobian()const;
    boost::shared_ptr<Eigen::VectorXd> getTaskFunction()const;
    boost::shared_ptr<Eigen::VectorXd> getTaskVelocity()const;


    boost::shared_ptr<std::vector<TaskObject> > getTaskObjects()const;

    //** Needs to be checked whether the task objects have appropriate properties in each derived task class */
    static boost::shared_ptr<Task> makeTask(unsigned int id, unsigned int priority, TaskType type, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics); ///<factory method
    //  virtual void setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs) = 0;

    //**Computes task function, velocity and jacobians. Assumes that the kinematics of the corresponding task objects have been computed prior to the call to this function.*/
    virtual void computeTask()=0;

    friend std::ostream& operator<<(std::ostream& str, Task const& task);

protected:
    Task(){};

    TaskType type_;
    unsigned int id_;
    unsigned int priority_;
    std::string sign_;
    unsigned int dim_;
    boost::shared_ptr<TaskDynamics> t_dynamics_;
    boost::shared_ptr<std::vector<TaskObject> > t_objs_;

    boost::shared_ptr<Eigen::MatrixXd> A_; ///< task jacobian
    boost::shared_ptr<Eigen::MatrixXd> E_; ///< task function state matrix, rows \in Task::dim_ correspond to the task function dynamics of the single task dimensions, columns \in Task::t_dynamics_->getDimension()+1 hold the corresponding derivatives
    ros::Time t_prev_;

    void updateTaskFunctionDerivatives();
};
//----------------------------------------------------------------
class ProjectPointPlane: public Task
{
public:
    ProjectPointPlane(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);
    //  virtual void setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs);
    virtual void computeTask();

protected:
    ProjectPointPlane(){};

private:
    //**Helper function to make sure that the given task objects are valid in the context of the task */
    void verifyTaskObjects();

    unsigned int jnt_index_; ///< index (in the TaskObject::joints_ vector) of the joint controlled by this task
};
//----------------------------------------------------------------
class JointSetpoint: public Task
{
public:
    JointSetpoint(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    virtual void computeTask();

protected:
    JointSetpoint(){};

private:
    //**Helper function to make sure that the given task objects are valid in the context of the task */
    void verifyTaskObjects();
    int jnt_index_;
};
//----------------------------------------------------------------
class JointVelocityLimits: public Task
{
public:
    JointVelocityLimits(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    virtual void computeTask();

protected:
    JointVelocityLimits(){};

private:
    //**Helper function to make sure that the given task objects are valid in the context of the task */
    void verifyTaskObjects();
    int jnt_index_;
};
//----------------------------------------------------------------
class ParallelLines: public Task
{
public:
    ParallelLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    virtual void computeTask();

protected:
    ParallelLines(){};

private:
    void verifyTaskObjects();
};
//----------------------------------------------------------------
class AngleLines: public Task
{
public:
   AngleLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

    virtual void computeTask();

protected:
    AngleLines(){};

private:
    void verifyTaskObjects();
};
//----------------------------------------------------------------
} //end namespace hqp_controllers

#endif // TASK_H
