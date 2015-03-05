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
        unsigned int d_dim = sqrt(data.rows());
        Eigen::MatrixXd A(d_dim,d_dim);
        unsigned int count = 0;
        for(unsigned int i=0; i<d_dim; i++)
            for(unsigned int j=0; j<d_dim; j++)
            {
                A(i,j) = data(count);
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
TaskDynamics::TaskDynamics() : d_dim_(0){}
//----------------------------------------------------------

unsigned int TaskDynamics::getDimension()const{return d_dim_;}
//----------------------------------------------------------
LinearTaskDynamics::LinearTaskDynamics() : TaskDynamics() {}
//----------------------------------------------------------
LinearTaskDynamics::LinearTaskDynamics(Eigen::MatrixXd const& A)
{
    setDynamicsMatrix(A);
}
//----------------------------------------------------------
void LinearTaskDynamics::setDynamicsMatrix(Eigen::MatrixXd const& A)
{
    ROS_ASSERT(A.cols()==A.rows());
    ROS_ASSERT(A.cols() > 0);

    A_ = A;
    d_dim_ = A_.cols();
}
//----------------------------------------------------------
Eigen::MatrixXd LinearTaskDynamics::getDynamicsMatrix()const{return A_;}
//----------------------------------------------------------
void LinearTaskDynamics::getDX(Eigen::VectorXd& dx, Eigen::VectorXd& x)const
{
    ROS_ASSERT( (dx.rows() == d_dim_) && (x.rows() == d_dim_) );

    dx=A_ * x;
}
//----------------------------------------------------------
}//end namespace hqp_controllers
