#ifndef TASK_H
#define TASK_H

#include <string>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>

namespace hqp_controllers {

enum TaskType {UNDEFINED=1, OBSTACLE_AVOIDANCE, IK_EE_PLANE};

class Task
{
public:
    Task();
    Task(std::string name, unsigned int id);

    std::string getName();
    unsigned int getId();
    TaskType getType();

    void setName(std::string);
    void setId(unsigned int id);

protected:
    std::string name_;
    unsigned int id_;
    TaskType type_;

    boost::scoped_ptr<Eigen::MatrixXd> A_; ///< task jacobian
    boost::scoped_ptr<Eigen::VectorXd> b_; ///< task reference

    virtual void computeTaskJacobian()=0;
    virtual void computeTaskReference()=0;

};

} //end namespace hqp_controllers

#endif // TASK_H
