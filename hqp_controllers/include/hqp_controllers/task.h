#ifndef TASK_H
#define TASK_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <hqp_controllers/task_link.h>
#include <hqp_controllers/task_dynamics.h>
#include <hqp_controllers_msgs/Task.h>

namespace hqp_controllers {
//----------------------------------------------------------------
enum TaskDescriptionFormat {ROS_MESSAGE = 1, XML = 2};
//----------------------------------------------------------------
enum TaskType {PROJECTION = 1, ORIENTATION = 2, JOINT_SETPOINT = 3, JOINT_LIMIT_AVOIDANCE = 4};
//----------------------------------------------------------------
class Task
{
public:
    Task(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    void setId(unsigned int id);
    void setPriority(unsigned int priority);
    void setIsEqualityTask(bool is_equality_task);
//    //  void setTaskDynamics(boost::shared_ptr<std::vector<TaskDynamics> > t_dynamics);
//    boost::shared_ptr<TaskDynamics> getTaskDynamics()const;
    unsigned int getId()const;

    unsigned int getDimension()const;
    unsigned int getPriority()const;
    bool getIsEqualityTask()const;

  Eigen::MatrixXd getTaskJacobian()const;
  Eigen::VectorXd getTaskFunction()const;
//    boost::shared_ptr<Eigen::VectorXd> getTaskVelocity()const;


//    boost::shared_ptr<std::vector<TaskLink> > getTaskLinks()const;

    //** Needs to be checked whether the task objects have appropriate properties in each derived task class */
    static boost::shared_ptr<Task> makeTask(void* description, TaskDescriptionFormat format); ///<factory method
//    //  virtual void setTaskLinks(std::pair<boost::shared_ptr<TaskLink>, boost::shared_ptr<TaskLink> > t_objs) = 0;

    //**Computes task function, velocity and jacobians. Also updates the kinematics of the corresponding task objects.*/
    virtual void updateTask()=0;

//    //** get the summed squared error of the task - should be implemented here, once the tasks are cleaned up */
//    virtual double getSSE()const=0;

//    friend std::ostream& operator<<(std::ostream& str, Task const& task);

protected:
    Task(){};

    unsigned int id_;
    unsigned int priority_;
    std::string task_frame_;
    bool is_equality_task_;

     unsigned int t_dim_; ///< task dimension == rows of the task jacobian Task::A_ / task function state matrix Task::E_
     boost::shared_ptr<TaskDynamics> t_dynamics_;
      std::vector<boost::shared_ptr<TaskLink> >  t_links_;


     Eigen::MatrixXd  A_; ///< task jacobian
     Eigen::MatrixXd  E_; ///< task function state matrix, rows \in Task::t_dim_ correspond to the task function dynamics of the single task dimensions, columns correspond to derivatives

     ros::Time t_prev_;
    bool t_start_;
    void updateTaskFunctionDerivatives();

};
//----------------------------------------------------------------
class Projection: public Task
{
public:
    Projection(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    //    virtual double getSSE()const;

protected:
    Projection(){};

private:


   // unsigned int jnt_index_; ///< index (in the TaskLink::joints_ vector) of the joint controlled by this task
};
//----------------------------------------------------------------
//class JointSetpoint: public Task
//{
//public:
//    JointSetpoint(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    JointSetpoint(){};

//private:
//    //**Helper function to make sure that the given task objects are valid in the context of the task */
//    void verifyTaskLinks();
//    int jnt_index_;
//};
////----------------------------------------------------------------
//class JointVelocityLimits: public Task
//{
//public:
//    JointVelocityLimits(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    JointVelocityLimits(){};

//private:
//    //**Helper function to make sure that the given task objects are valid in the context of the task */
//    void verifyTaskLinks();
//    int jnt_index_;
//};
////----------------------------------------------------------------
//class ParallelLines: public Task
//{
//public:
//    ParallelLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    ParallelLines(){};

//private:
//    void verifyTaskLinks();
//};
////----------------------------------------------------------------
//class AngleLines: public Task
//{
//public:
//   AngleLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    AngleLines(){};

//private:
//    void verifyTaskLinks();
//};
////----------------------------------------------------------------
//class ProjectPointCylinder: public Task
//{
//public:
//    ProjectPointCylinder(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);
//    //  virtual void setTaskLinks(std::pair<boost::shared_ptr<TaskLink>, boost::shared_ptr<TaskLink> > t_objs);
//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    ProjectPointCylinder(){};

//private:
//    //**Helper function to make sure that the given task objects are valid in the context of the task */
//    void verifyTaskLinks();
//};
////----------------------------------------------------------------
//class CoplanarLines: public Task
//{
//public:
//    CoplanarLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);

//    virtual void computeTask();
//    virtual double getSSE()const;

//protected:
//    CoplanarLines(){};

//private:
//    void verifyTaskLinks();
//};
////----------------------------------------------------------------
//class ProjectLineLine: public Task
//{
//public:
//    ProjectLineLine(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);
//    //  virtual void setTaskLinks(std::pair<boost::shared_ptr<TaskLink>, boost::shared_ptr<TaskLink> > t_objs);
//    virtual void computeTask();
//        virtual double getSSE()const;

//protected:
//    ProjectLineLine(){};

//private:
//    //**Helper function to make sure that the given task objects are valid in the context of the task */
//    void verifyTaskLinks();
//};
////----------------------------------------------------------------
//class ProjectSpherePlane: public Task
//{
//public:
//    ProjectSpherePlane(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskLink> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics);
//    //  virtual void setTaskLinks(std::pair<boost::shared_ptr<TaskLink>, boost::shared_ptr<TaskLink> > t_objs);
//    virtual void computeTask();
//        virtual double getSSE()const;

//protected:
//    ProjectSpherePlane(){};

//private:
//    //**Helper function to make sure that the given task objects are valid in the context of the task */
//    void verifyTaskLinks();

//   // unsigned int jnt_index_; ///< index (in the TaskLink::joints_ vector) of the joint controlled by this task
//};
//----------------------------------------------------------------
} //end namespace hqp_controllers

#endif // TASK_H
