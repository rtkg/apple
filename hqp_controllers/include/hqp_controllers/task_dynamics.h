#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

namespace hqp_controllers{
//----------------------------------------------------------
enum TaskDynamicsType { LINEAR_DYNAMICS = 1};
//----------------------------------------------------------
class TaskDynamics
{
public:

    TaskDynamics();

    unsigned int getDimension()const;
    virtual void getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const = 0;
    static boost::shared_ptr<TaskDynamics>  makeTaskDynamics(TaskDynamicsType type, Eigen::VectorXd const& data); ///<factory method

protected:

    unsigned int d_dim_; ///< dimension of the state space

};
//----------------------------------------------------------
class LinearTaskDynamics: public TaskDynamics
{
public:
    LinearTaskDynamics();
    LinearTaskDynamics(Eigen::MatrixXd const& A);

    void setDynamicsMatrix(Eigen::MatrixXd const& A);
    Eigen::MatrixXd getDynamicsMatrix()const;
    virtual void getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const;

protected:

    Eigen::MatrixXd A_; ///<dynamics matrix dx=A_* x
};
//----------------------------------------------------------

}//end namespace hqp_controllers

#endif
