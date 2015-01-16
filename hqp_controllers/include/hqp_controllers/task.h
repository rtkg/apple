#ifndef TASK_H
#define TASK_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace hqp_controllers {

enum TaskType {UNDEFINED_TASK=1, OBSTACLE_AVOIDANCE, IK_EE_PLANE};

class Task
{
public:
    Task();
    Task(std::string name, unsigned int id);
    Task(std::string name, std::string frame, unsigned int id);

    std::string getName();
    std::string getFrame();
    unsigned int getId();
    TaskType getType();
    bool isInitialized();
    unsigned int getPriority();

    void setName(std::string);
    void setId(unsigned int id);
    void init();
    void setPriority(unsigned int priority);

protected:
    unsigned int priority_;
    std::string name_;
    std::string frame_; ///< parent frame associated with this task
    unsigned int id_;
    TaskType type_;
    bool initialized_;

    boost::shared_ptr<Eigen::MatrixXd> A_; ///< task function jacobian
    boost::shared_ptr<Eigen::VectorXd> b_; ///< task function reference

    virtual void computeTaskJacobian()=0;
    virtual void computeTaskReference()=0;

};

} //end namespace hqp_controllers

#endif // TASK_H
