#include <hqp_controllers/hqp.h>

namespace hqp_controllers {

//-----------------------------------------
HQPStage::HQPStage() : dim_(0)
{
    de_.reset(new Eigen::VectorXd);
    signs_.reset(new char);
    A_.reset(new Eigen::MatrixXd);
}
//-----------------------------------------
HQPStage::HQPStage(Task const& task)
{
    de_ = task.getTaskVelocity();
    A_ = task.getTaskJacobian();


//    de_.reset(new Eigen::VectorXd(task.getTaskVelocity();));
//    signs_.reset(new char[]);
//    A_.reset(new Eigen::MatrixXd(A));
}
//-----------------------------------------
void HQPStage::appendTask(Task const& task)
{

}
//-----------------------------------------
}//end namespace hqp_controllers
