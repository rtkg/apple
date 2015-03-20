#ifndef TASK_H
#define TASK_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <hqp_controllers/task_link.h>
#include <hqp_controllers/task_dynamics.h>
#include <hqp_controllers_msgs/Task.h>
#include <kdl/tree.hpp>

namespace hqp_controllers {
//----------------------------------------------------------------
enum TaskDescriptionFormat {ROS_MESSAGE = 1, XML = 2};
//----------------------------------------------------------------
enum TaskType {PROJECTION = 1, PARALLEL = 2, JOINT_SETPOINT = 3, JOINT_LIMIT_AVOIDANCE = 4, COPLANAR = 5};
//----------------------------------------------------------------
class Task
{
public:
    Task(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    void setId(unsigned int id);
    void setPriority(unsigned int priority);
    void setIsEqualityTask(bool is_equality_task);
    void setTaskVelocityDamping(double ds, double di);
    //    //  void setTaskDynamics(boost::shared_ptr<std::vector<TaskDynamics> > t_dynamics);
    //    boost::shared_ptr<TaskDynamics> getTaskDynamics()const;
    unsigned int getId()const;

    unsigned int getPriority()const;
    bool getIsEqualityTask()const;
    void setName(std::string name);
    std::string getName()const;

    Eigen::MatrixXd getTaskJacobian()const;
    Eigen::VectorXd getTaskFunction()const;
    Eigen::VectorXd getTaskVelocity()const;

    std::vector<boost::shared_ptr<TaskLink> > getTaskLinks()const;

    //** Needs to be checked whether the task objects have appropriate properties in each derived task class */
    static boost::shared_ptr<Task> makeTask(unsigned int id, XmlRpc::XmlRpcValue& t_description, KDL::Tree const& k_tree, std::vector< hardware_interface::JointHandle > const& joints); ///<factory method
    static XmlRpc::XmlRpcValue taskMessageToXmlRpcValue(hqp_controllers_msgs::Task const& msg);
    //    //  virtual void setTaskLinks(std::pair<boost::shared_ptr<TaskLink>, boost::shared_ptr<TaskLink> > t_objs) = 0;

    //**Computes task function, velocity and jacobians. Also updates the kinematics of the corresponding task objects.*/
    virtual void updateTask()=0;

    //    //** get the summed squared error of the task - should be implemented here, once the tasks are cleaned up */
    virtual double getTaskProgress()const=0;

    friend std::ostream& operator<<(std::ostream& str, Task const& task);

protected:
    Task(){};

    unsigned int id_;
    unsigned int priority_;
    std::string task_frame_;
    bool is_equality_task_;
    boost::shared_ptr<TaskDynamics> t_dynamics_;
    std::vector<boost::shared_ptr<TaskLink> >  t_links_;
    Eigen::MatrixXd  J_; ///< task jacobian
    Eigen::MatrixX2d  E_; ///< task function state matrix, rows \in Task::t_dim_ correspond to the task function dynamics of the single task dimensions, columns correspond to derivatives
    //    ros::Time t_prev_;
    //    bool t_start_;
    double ds_;
    double di_;
    std::string name_;

    //    void updateTaskFunctionDerivatives();
    void computeTaskLinkKinematics();
};
//----------------------------------------------------------------
class Projection: public Task
{
public:
    Projection(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    virtual double getTaskProgress()const;

protected:
    Projection(){};

private:

};
//----------------------------------------------------------------
class Parallel: public Task
{
public:
    Parallel(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    virtual double getTaskProgress()const;

protected:
    Parallel(){};

private:

};
//----------------------------------------------------------------
class Coplanar: public Task
{
public:
    Coplanar(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    virtual double getTaskProgress()const;

protected:
    Coplanar(){};

private:

};
//----------------------------------------------------------------
class JointSetpoint: public Task
{
public:
    JointSetpoint(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    virtual double getTaskProgress()const;

protected:
    JointSetpoint(){};

};
//----------------------------------------------------------------
class JointLimitAvoidance: public Task
{
public:
    JointLimitAvoidance(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links);

    virtual void updateTask();
    virtual double getTaskProgress()const;

protected:
    JointLimitAvoidance(){};

};
//----------------------------------------------------------------
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
