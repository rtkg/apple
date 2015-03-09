#include <hqp_controllers/task_link.h>
#include <hqp_controllers/conversions.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <ros/ros.h>
#include <hqp_controllers/utilities.h>
#include <sstream>

namespace hqp_controllers {

//----------------------------------------------------
std::ostream& operator<<(std::ostream& str, TaskLink const& link)
{
    str<<"TASK LINK: "<<std::endl;
    str<<"link frame: "<<link.link_frame_<<std::endl;
    str<<"task frame: "<<link.task_frame_<<std::endl;

    for(unsigned int i=0; i<link.geometries_.size();i++)
        str<< *(link.geometries_.at(i))<<std::endl;

    str<<std::endl;
}
//----------------------------------------------------
TaskLink::TaskLink(std::string const& task_frame, KDL::Chain const& chain, std::vector< hardware_interface::JointHandle > const& joints) : task_frame_(task_frame), chain_(chain), joints_(joints)
{

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    j_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    computeJointMap();

    n_jnts_ = joints_.size();
    jac_.resize(6,n_jnts_);
    jac_.setZero();
    chain_jac_.resize(n_jnts_);

    if(chain_.getNrOfSegments() == 0)
        link_frame_ = task_frame_;
    else
        link_frame_ = chain_.segments.back().getName();
}
//----------------------------------------------------
unsigned int TaskLink::getNumJoints(){return n_jnts_;}
//----------------------------------------------------
void TaskLink::computeJointMap()
{

    unsigned int n_jnts = joints_.size();
    unsigned int n_chain_jnts = chain_.getNrOfJoints();
    unsigned int n_chain_sgmnts = chain_.getNrOfSegments(); //if there are locked joints, the segment nr != the joint nr
    joint_map_.resize(n_chain_jnts);

    unsigned int k=0; //index of the unlocked joints
    for(unsigned int i=0; i<n_chain_sgmnts; i++)//iterate over all chain segments
    {
        for (unsigned int j=0; j<n_jnts; j++)//iterate over all controlled joints
        {
            if(chain_.getSegment(i).getJoint().getName() == joints_.at(j).getName())
            {
                joint_map_(k)=j;//associate the unlocked chain joints with the corresponding controlled joints
                k++;
            }
        }
    }

    // ============= DEBUG PRINT =============================
    //    std::cout<<"CHAIN JOINTS"<<std::endl;
    //    for(unsigned int i=0; i<n_chain_sgmnts; i++)
    //    {
    //        std::cout<<chain_->getSegment(i).getJoint().getName()<<std::endl;
    //    }
    //    std::cout<<std::endl<<"CONTROLLED JOINTS"<<std::endl;
    //    for (unsigned int j=0; j<n_jnts; j++)
    //    {
    //        std::cout<<joints_->at(j).getName()<<std::endl;
    //    }
    //    std::cout<<std::endl<<"JOINT MAP"<<std::endl<<(*joint_map_).transpose()<<std::endl;
    // ============= DEBUG PRINT END =============================
}
//----------------------------------------------------
//std::string TaskLink::getLink() const{return link_;}
//----------------------------------------------------
//boost::shared_ptr<KDL::Chain> TaskLink::getChain() const {return chain_;}
//----------------------------------------------------
//boost::shared_ptr<std::vector< hardware_interface::JointHandle > > TaskLink::getJoints() const {return joints_;}
//----------------------------------------------------
void TaskLink::computeKinematics()
{
        //read the chain joint positions from the controlled joints
        unsigned int n_jnts = joint_map_.rows();

        KDL::JntArray q(n_jnts);
        for(unsigned int i=0; i<n_jnts; i++)
            q.data(i) = joints_.at(joint_map_(i)).getPosition();

        //compute the chain jacobian
        KDL::Frame pose;
        if(fk_solver_->JntToCart(q,pose) < 0)
            ROS_ERROR("Could not compute forward kinematics of task link with link frame %s.",link_frame_.c_str());

        if(j_solver_->JntToJac(q, chain_jac_) < 0)
            ROS_ERROR("Could not compute jacobian of task link with link frame %s.",link_frame_.c_str());

        KDLToEigen(pose, T_l_t_);

        //map the chain jacobian to the controlled joint jacobian
        jac_.setZero();
        for(unsigned int i=0; i<n_jnts; i++)
            jac_.col(joint_map_(i)) = chain_jac_.data.col(i);

        //Transform the associated geometries
        for(unsigned int i=0; i<geometries_.size(); i++)
            geometries_.at(i)->transformTaskData(T_l_t_);

    //        std::cout<<"number of joints: "<<n_jnts<<std::endl;
    //        std::cout<<"Task object w. root: "<<root_<<", link: "<<link_<<" and id: "<<id_<<std::endl;
    //        std::cout<<"Pose translation: "<<std::endl<< trans_l_r_->translation().transpose()<<std::endl;
    //        std::cout<<"Pose rotation: "<<std::endl<< trans_l_r_->rotation()<<std::endl;
    //        std::cout<<"Chain jacobian: "<<std::endl<<chain_jacobian_->data<<std::endl;
    //        std::cout<<"Jacobian: "<<std::endl<< *jacobian_<<std::endl;
    //        std::cout<<std::endl;

    //    //========= DEBUG PRINT CHAIN ==========
    //    std::ostringstream out;
    //    printKDLChain(out, *chain_);
    //    std::cout<<"CHAIN:"<<std::endl;
    //    std::cout<<out.str()<<std::endl;
    //    //========= DEBUG PRINT CHAIN END ==========

}
//----------------------------------------------------
Eigen::Affine3d TaskLink::getLinkTransform() const {return T_l_t_;}
//----------------------------------------------------
Eigen::MatrixXd TaskLink::getJacobian() const {return jac_;}
//----------------------------------------------------
Eigen::MatrixXd TaskLink::getJacobian(Eigen::Vector3d& base_AB) const
{
    //change the reference point
    KDL::Jacobian c_jac = chain_jac_;
    c_jac.changeRefPoint(KDL::Vector(base_AB(0), base_AB(1), base_AB(2)));

    //map to the controlled joints jacobian
    Eigen::MatrixXd jac(6, n_jnts_);
    jac.setZero();
    for(unsigned int i=0; i<joint_map_.rows(); i++)
        jac.col(joint_map_(i)) = c_jac.data.col(i);

    return jac;
}
//----------------------------------------------------
//boost::shared_ptr<KDL::Jacobian> TaskLink::getChainJacobian() const {return chain_jacobian_;}
//----------------------------------------------------
void TaskLink::addGeometry(boost::shared_ptr<TaskGeometry> geometry)
{
    //Make sure link and root frames correspond
    ROS_ASSERT(link_frame_ == geometry->getLinkFrame());
    ROS_ASSERT(task_frame_ == geometry->getTaskFrame());

    geometries_.push_back(geometry);
}
//----------------------------------------------------
std::vector<boost::shared_ptr<TaskGeometry> > TaskLink::getGeometries() const {return geometries_;}
//----------------------------------------------------
} //end namespace hqp_controllers