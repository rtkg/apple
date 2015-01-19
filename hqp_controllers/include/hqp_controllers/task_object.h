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
    TaskObject(unsigned int id, boost::shared_ptr<KDL::Chain> chain, boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints);

    void addGeometry(boost::shared_ptr<TaskGeometry> geometry);
    void setChain(boost::shared_ptr<KDL::Chain> chain);
    void setJoints(boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints);
    void setId(unsigned int id);

    unsigned int getId() const;
    std::string getFrame() const;
    boost::shared_ptr<KDL::Chain> getChain() const;
    boost::shared_ptr<std::vector< hardware_interface::JointHandle > > getJoints() const;
    boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > getGeometries() const;
    boost::shared_ptr<Eigen::Affine3d> getPose() const;
    boost::shared_ptr<Eigen::MatrixXd> getJacobian() const;
    boost::shared_ptr<KDL::Jacobian> getChainJacobian() const;
   /** Computes the the pose of the task object (forward kinematics) and the jacobian.
    Input: joint angles q*/
    void computeKinematics();

private:
    void updateJointMap();

    unsigned int id_;
    std::string frame_;
    boost::shared_ptr<Eigen::Affine3d> pose_;   ///< pose of the task object in the root frame (the chain base frame)
    boost::shared_ptr<Eigen::MatrixXd> jacobian_; ///< full jacobian of the task object frame
    boost::shared_ptr<KDL::Jacobian> chain_jacobian_; ///< jacobian for only those (unlocked) joints participating in the TaskObject::chain_
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> j_solver_;
    boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > geometries_;
    boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints_;
    boost::shared_ptr<KDL::Chain> chain_;
    boost::shared_ptr<Eigen::VectorXi> joint_map_; ///< associates the members of TaskObject::joints_ to the (unlocked) joints participating in the TaskObject::chain_, e.g., joint_map_ = [4, 2, 0]^T means that TaskObject::chain_ joints 0,1,2 correspond to the controlled joints 4,2,0 respectively
};

#endif // TASK_OBJECT_H

} //end namespace hqp_controllers
