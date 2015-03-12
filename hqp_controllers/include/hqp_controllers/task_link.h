#ifndef TASK_LINK_H
#define TASK_LINK_H

#include <hqp_controllers/task_geometry.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <hardware_interface/joint_command_interface.h>

namespace hqp_controllers {

class TaskLink
{
public:

    TaskLink(std::string const& task_frame, KDL::Chain const& chain, std::vector< hardware_interface::JointHandle > const& joints);

    void addGeometry(boost::shared_ptr<TaskGeometry> geometry);


    //std::string getLink() const;

    KDL::Chain getChain() const;
    std::vector< hardware_interface::JointHandle> getJoints() const;
    std::vector<boost::shared_ptr<TaskGeometry> >  getGeometries() const;
    Eigen::Affine3d getLinkTransform() const;
    Eigen::MatrixXd getJacobian() const;
    //**Changes the reference point of the task object jacobian by the vector from base_AB, which is the vector from the old ref point to the new ref point expressed in the jacobian root frame */
    Eigen::MatrixXd getJacobian(Eigen::Vector3d& base_AB) const;
    //boost::shared_ptr<KDL::Jacobian> getChainJacobian() const;
    unsigned int getNumJoints();
    /** Computes the the pose of the task link (forward kinematics) and the jacobian.*/
    void computeKinematics();
    Eigen::VectorXi getJointMap()const;
    //double getJointValue()const;

    friend std::ostream& operator<<(std::ostream& str, TaskLink const& obj);

private:
    TaskLink(){};

    void computeJointMap();

    std::string link_frame_;
    std::string task_frame_;
    Eigen::Affine3d T_l_t_;   ///< transformation from the link frame to the task frame (= the pose of the link frame expressed in the task frame)
    Eigen::MatrixXd jac_; ///< full jacobian of all controlled joints
    KDL::Jacobian chain_jac_; ///< jacobian for only those (unlocked) joints participating in the TaskObject::chain_
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> j_solver_;
    std::vector<boost::shared_ptr<TaskGeometry> > geometries_;
    std::vector< hardware_interface::JointHandle > joints_;
    KDL::Chain chain_;
    Eigen::VectorXi joint_map_; ///< associates the members of TaskLink::joints_ to the (unlocked) joints participating in the TaskObject::chain_, e.g., joint_map_ = [4, 2, 0]^T means that TaskLink::chain_ joints 0,1,2 correspond to the controlled joints 4,2,0 respectively
    unsigned int n_jnts_; ///< number of controlled joints
};

} //end namespace hqp_controllers

#endif // TASK_LINK_H
