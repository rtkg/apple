#include <hqp_controllers/hqp.h>

namespace hqp_controllers {

//-----------------------------------------
std::ostream& operator<<(std::ostream& str, HQPStage const& stage)
{
    str<<"HQPSTAGE"<<std::endl;
    str<<"dim_: "<<stage.dim_<<std::endl;
    str<<"de_: "<<stage.de_->transpose()<<std::endl;
    str<<"signs: ";
    for(unsigned int i=0; i<stage.signs_->size();i++)
        str<<stage.signs_->at(i)<<" ";

    str<<std::endl<<"A_:"<<std::endl<<(*stage.A_)<<std::endl;
}
//-----------------------------------------
HQPStage::HQPStage() : dim_(0)
{
    de_.reset(new Eigen::VectorXd);
    signs_.reset(new std::vector<std::string>);
    A_.reset(new Eigen::MatrixXd);
}
//-----------------------------------------
HQPStage::HQPStage(Task const& task)
{
    dim_ = task.getDimension();
    de_ = task.getTaskVelocity();
    A_ = task.getTaskJacobian();
    signs_.reset(new std::vector<std::string>(dim_));
    std::fill(signs_->begin(), signs_->end(), task.getSign());
}
//-----------------------------------------
void HQPStage::appendTask(Task const& task)
{
    unsigned int dim = task.getDimension();
    de_->conservativeResize(dim_ + dim);
    A_->conservativeResize(dim + dim_, Eigen::NoChange);
    de_->tail(dim) = (*task.getTaskVelocity());
    A_->bottomRows(dim) = (*task.getTaskJacobian());
    for(unsigned int i=0; i<dim; i++)
        signs_->push_back(task.getSign());

    dim_ = dim_ +dim;
}
//-----------------------------------------
HQPSolver::HQPSolver(){}
//-----------------------------------------
bool HQPSolver::solve(const std::map<unsigned int, boost::shared_ptr<HQPStage> > &hqp)const
{
     //   env_lock_.lock();
GRBModel solver(env_);
   //     env_lock_.unlock();
}
//-----------------------------------------
}//end namespace hqp_controllers
