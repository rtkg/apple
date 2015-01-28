#ifndef HQP_H
#define HQP_H

#include <hqp_controllers/task.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <gurobi_c++.h>

namespace hqp_controllers {
//--------------------------------------------------------------
struct HQPStage
{
    HQPStage();
    HQPStage(Task const& task);

    void appendTask(Task const& task);

    boost::shared_ptr<Eigen::VectorXd> de_;
    boost::shared_ptr<std::vector<std::string> > signs_;
    boost::shared_ptr<Eigen::MatrixXd> A_;

    unsigned int dim_;

     friend std::ostream& operator<<(std::ostream& str, HQPStage const& stage);
};
//--------------------------------------------------------------
class HQPSolver
{
   public:

    HQPSolver();

    bool solve(std::map<unsigned int, boost::shared_ptr<HQPStage> > const& hqp)const;
    Eigen::VectorXd getSolution()const;

private:

    GRBEnv env_;
   // boost::mutex env_lock_;
    Eigen::VectorXd solution_;

};
//--------------------------------------------------------------
}//end namespace hqp_controllers

#endif
