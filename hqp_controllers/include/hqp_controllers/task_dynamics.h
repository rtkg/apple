#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>


namespace hqp_controllers{
//----------------------------------------------------------
enum TaskDynamicsType {UNDEFINED_DYNAMICS = 0, LINEAR_DYNAMICS = 1};
//----------------------------------------------------------
class TaskDynamics
{
public:

    TaskDynamics();

    TaskDynamicsType getType()const;
    unsigned int getDimension()const;
    virtual void getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const = 0;
    static boost::shared_ptr<TaskDynamics>  makeTaskDynamics(TaskDynamicsType type, Eigen::VectorXd const& data); ///<factory method
protected:

    TaskDynamicsType type_;
    unsigned int dim_; ///< dimension of the state space

};
//----------------------------------------------------------
class LinearTaskDynamics: public TaskDynamics
{
public:
    LinearTaskDynamics();
    LinearTaskDynamics(boost::shared_ptr<Eigen::MatrixXd> A);

    void setDynamicsMatrix(boost::shared_ptr<Eigen::MatrixXd> A);
    boost::shared_ptr<Eigen::MatrixXd> getDynamicsMatrix()const;

     virtual void getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const;

protected:

    boost::shared_ptr<Eigen::MatrixXd> A_; ///<dynamics matrix dx=A_* x
};
//----------------------------------------------------------

}//end namespace hqp_controllers

#endif
