#ifndef HQP_H
#define HQP_H

#include <hqp_controllers/task.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

namespace hqp_controllers {
//--------------------------------------------------------------
struct HQPStage
{
    HQPStage();
    HQPStage(Task const& task);

    void appendTask(Task const& task);

    boost::shared_ptr<Eigen::VectorXd> de_;
    boost::shared_ptr<char []> signs_;
    boost::shared_ptr<Eigen::MatrixXd> A_;

    unsigned int dim_;
};
//--------------------------------------------------------------


}//end namespace hqp_controllers

#endif
