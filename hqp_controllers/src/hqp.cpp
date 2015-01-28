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
Eigen::VectorXd HQPSolver::getSolution()const{return solution_;}
//-----------------------------------------
bool HQPSolver::solve(const std::map<unsigned int, boost::shared_ptr<HQPStage> > &hqp)const
{
     //   env_lock_.lock();
//GRBModel solver(env_);
GRBModel model(env_);

int nVar=3;
int nConstraints=2;
GRBVar* x =model.addVars(NULL,NULL,NULL,NULL,NULL,nVar);
model.update();

GRBLinExpr* lhsides=new GRBLinExpr[nConstraints];
char* senses=new char[nConstraints]; std::fill_n(senses,nConstraints,GRB_LESS_EQUAL);
double* rhsides=new double[nConstraints]; std::fill_n(rhsides,nConstraints,0);

double* coeff=new double[nVar]; std::fill_n(coeff,nVar,0);
for (uint i=0; i<nConstraints;i++)
  lhsides[i].addTerms(coeff,x,nVar);

try
  {
    GRBConstr* constrs=model.addConstrs(lhsides,senses,rhsides,NULL,nConstraints);
  }
catch(GRBException e)
  {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  }

delete[] x;
delete[] lhsides;
delete[] senses;
delete[] rhsides;
delete[] coeff;

std::cout<<"solved HQP! Yay!"<<std::endl;

return 0;
   //     env_lock_.unlock();
}
//-----------------------------------------
}//end namespace hqp_controllers
