#ifndef HQP_H
#define HQP_H

#include <hqp_controllers/task.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <gurobi_c++.h>

namespace hqp_controllers {
//--------------------------------------------------------------
#define TIKHONOV_FACTOR  1e-4
#define PRESOLVE         -1
#define OPTIMALITY_TOL   1e-6
#define SCALE_FLAG       1
#define TIME_LIMIT       1
    //0.005
#define OUTPUT_FLAG      0
#define DUAL_REDUCTIONS  1
//--------------------------------------------------------------
struct HQPStage
{
    HQPStage(Task const& task);
    void appendTask(Task const& task);

    Eigen::VectorXd de_; ///<desired task velocities
    Eigen::VectorXd x_; ///<HQP solution for this stage
    Eigen::VectorXd w_; ///<slack variables for this stage
    std::vector<bool> is_equalities_;
    Eigen::MatrixXd J_;

    unsigned int s_dim_;
    bool solved_;

    friend std::ostream& operator<<(std::ostream& str, HQPStage const& stage);

private:
        HQPStage(){};
};
//--------------------------------------------------------------
class HQPSolver
{
public:

    HQPSolver();

    bool solve(std::map<unsigned int, boost::shared_ptr<HQPStage> >& hqp);

private:

    GRBEnv env_;
    // boost::mutex env_lock_;
   // Eigen::VectorXd solution_;

    Eigen::VectorXd b_;
    Eigen::VectorXd w_;
    Eigen::MatrixXd A_;
//    Eigen::VectorXd x_;
    std::vector<char> senses_;

    void reset();
};
//--------------------------------------------------------------
}//end namespace hqp_controllers

#endif
