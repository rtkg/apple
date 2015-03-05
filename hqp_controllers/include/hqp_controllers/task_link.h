#ifndef TASK_LINK_H
#define TASK_LINK_H

#include <hqp_controllers/task_geometry.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <hardware_interface/joint_command_interface.h>

namespace hqp_controllers {

class TaskLink
{
public:
    TaskLink();
    TaskLink(KDL::Chain const& chain, std::vector< hardware_interface::JointHandle > const& joints);

    //void addGeometry(boost::shared_ptr<TaskGeometry> geometry);
    //void setChain(boost::shared_ptr<KDL::Chain> chain, std::string root);
    //void setJoints(boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints);



    //std::string getLink() const;

   // boost::shared_ptr<KDL::Chain> getChain() const;
//    boost::shared_ptr<std::vector< hardware_interface::JointHandle > > getJoints() const;
//    boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > getGeometries() const;
  //  boost::shared_ptr<Eigen::Affine3d> getLinkTransform() const;
//    boost::shared_ptr<Eigen::MatrixXd> getJacobian() const;
    //**Changes the reference point of the task object jacobian by the vector from base_AB, which is the vector from the old ref point to the new ref point expressed in the jacobian root frame */
  //  boost::shared_ptr<Eigen::MatrixXd> getJacobian(Eigen::Vector3d& base_AB) const;
    //boost::shared_ptr<KDL::Jacobian> getChainJacobian() const;
   /** Computes the the pose of the task object (forward kinematics) and the jacobian.*/
    void computeKinematics();

    friend std::ostream& operator<<(std::ostream& str, TaskLink const& obj);

private:
    void updateJointMap();

    std::string link_;
    Eigen::Affine3d l_r_T_;   ///< transformation from the link frame to the root frame (= the pose of the link frame expressed in the root frame)
    Eigen::MatrixXd jac_; ///< full jacobian of all controlled joints
    KDL::Jacobian chain_jac_; ///< jacobian for only those (unlocked) joints participating in the TaskObject::chain_
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> j_solver_;
    std::vector<boost::scoped_ptr<TaskGeometry> > geometries_;
    std::vector< hardware_interface::JointHandle > joints_;
    KDL::Chain chain_;
    Eigen::VectorXi joint_map_; ///< associates the members of TaskObject::joints_ to the (unlocked) joints participating in the TaskObject::chain_, e.g., joint_map_ = [4, 2, 0]^T means that TaskObject::chain_ joints 0,1,2 correspond to the controlled joints 4,2,0 respectively
};

} //end namespace hqp_controllers

#endif // TASK_LINK_H
