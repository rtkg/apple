#ifndef TASK_OBJECT_H
#define TASK_OBJECT_H

#include <hqp_controllers/task_geometry.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <hardware_interface/joint_command_interface.h>

namespace hqp_controllers {

class TaskObject
{
public:

    TaskObject();
    TaskObject(std::string frame, boost::shared_ptr<KDL::Chain> chain);

    void setFrame(std::string frame);
    void setChain(boost::shared_ptr<KDL::Chain> chain);
    void addGeometry(boost::shared_ptr<TaskGeometry> geometry);

    std::string getFrame() const;
    boost::shared_ptr<KDL::Chain> getChain() const;
    boost::shared_ptr<Eigen::Affine3d> getPose() const;
    boost::shared_ptr<Eigen::MatrixXd> getJacobian() const;
    boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > getGeometries() const;
   /** Computes the the pose of the task object (forward kinematics) and the jacobian.
    Input: joint angles q*/
    void computeKinematics(const std::vector< hardware_interface::JointHandle >& joints);

private:

    std::string frame_; ///< frame to which the task object is attached
    boost::shared_ptr<Eigen::Affine3d> pose_;   ///< pose of the task object in the root frame (the chain base frame)
    boost::shared_ptr<Eigen::MatrixXd> jacobian_; ///< jacobian of the task object frame
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> j_solver_;
    boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > geometries_;
    boost::shared_ptr<KDL::Chain> chain_;
};

#endif // TASK_OBJECT_H

} //end namespace hqp_controllers
