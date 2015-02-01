#include <hqp_controllers/task_dynamics.h>
#include <math.h>

namespace hqp_controllers{
//----------------------------------------------------------
boost::shared_ptr<TaskDynamics>  TaskDynamics::makeTaskDynamics(TaskDynamicsType type, Eigen::VectorXd const& data)
{
    boost::shared_ptr<TaskDynamics> dynamics;

    if(type == LINEAR_DYNAMICS)
    {
        //Make sure the data dimensions are suitable for a square matrix
        ROS_ASSERT( fmod(data.rows(), sqrt(data.rows())) == 0);
        unsigned int dim = sqrt(data.rows());
        boost::shared_ptr<Eigen::MatrixXd> A(new Eigen::MatrixXd(dim,dim));
        unsigned int count = 0;
        for(unsigned int i=0; i<dim; i++)
            for(unsigned int j=0; j<dim; j++)
            {
                (*A)(i,j) = data(count);
                count++;
            }
        dynamics.reset(new LinearTaskDynamics(A));
    }
    else
    {
        ROS_ERROR("Task geometry type %d is invalid.",type);
        ROS_BREAK();
    }
    return dynamics;
}
//----------------------------------------------------------
TaskDynamics::TaskDynamics() : type_(UNDEFINED_DYNAMICS), dim_(0){}
//----------------------------------------------------------

unsigned int TaskDynamics::getDimension()const{return dim_;}
//----------------------------------------------------------
TaskDynamicsType TaskDynamics::getType()const{return type_;}
//----------------------------------------------------------
LinearTaskDynamics::LinearTaskDynamics() : TaskDynamics()
{
    type_ = LINEAR_DYNAMICS;
}
//----------------------------------------------------------
LinearTaskDynamics::LinearTaskDynamics(boost::shared_ptr<Eigen::MatrixXd> A) :  A_(A)
{
    type_ = LINEAR_DYNAMICS;
    setDynamicsMatrix(A);
}
//----------------------------------------------------------
void LinearTaskDynamics::setDynamicsMatrix(boost::shared_ptr<Eigen::MatrixXd> A)
{
    ROS_ASSERT(A->cols()==A->rows());
    ROS_ASSERT(A->cols() > 0);

    A_ = A;
    dim_ = A_->cols();
}
//----------------------------------------------------------
boost::shared_ptr<Eigen::MatrixXd> LinearTaskDynamics::getDynamicsMatrix()const{return A_;}
//----------------------------------------------------------
void LinearTaskDynamics::getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const
{
    ROS_ASSERT( (dx.rows() == dim_) && (x.rows() == dim_) );

    dx=(*A_) * x;
}
//----------------------------------------------------------
}//end namespace hqp_controllers
