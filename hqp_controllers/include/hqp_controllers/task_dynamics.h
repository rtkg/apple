#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

namespace hqp_controllers{

//----------------------------------------------------------
class TaskDynamics
{
public:

    TaskDynamics();
    TaskDynamics(unsigned int dim);

    virtual void computeDX()const=0;
    unsigned int getDimension()const;

protected:

    unsigned int dim_;

};
//----------------------------------------------------------
class LinearTaskDynamics: public TaskDynamics
{
public:
    LinearTaskDynamics(unsigned int dim);

    virtual void computeDX() const;
};
//----------------------------------------------------------

}//end namespace hqp_controllers

#endif
