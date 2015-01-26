#include <hqp_controllers/task_dynamics.h>

namespace hqp_controllers{
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
