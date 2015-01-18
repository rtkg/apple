#include <hqp_controllers/task_object.h>
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
TaskObject::TaskObject()
{
    pose_.reset(new Eigen::Affine3d);
    jacobian_.reset(new Eigen::MatrixXd);
    chain_jacobian_.reset(new KDL::Jacobian);
    chain_.reset(new KDL::Chain);
    geometries_.reset(new std::vector<boost::shared_ptr<TaskGeometry> >);
    joints_.reset(new std::vector<hardware_interface::JointHandle>);
    joint_map_.reset(new Eigen::VectorXi);
}
//----------------------------------------------------
TaskObject::TaskObject(boost::shared_ptr<KDL::Chain> chain, boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints) : chain_(chain), joints_(joints)
{    pose_.reset(new Eigen::Affine3d);
     jacobian_.reset(new Eigen::MatrixXd);
      chain_jacobian_.reset(new KDL::Jacobian);
       geometries_.reset(new std::vector<boost::shared_ptr<TaskGeometry> >);
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(*chain_));
         j_solver_.reset(new KDL::ChainJntToJacSolver(*chain_));

          updateJointMap();
}
//----------------------------------------------------
void TaskObject::updateJointMap()
{
    unsigned int n_jnts = joints_->size();
    unsigned int n_chain_jnts = chain_->getNrOfJoints();
    unsigned int n_chain_sgmnts = chain_->getNrOfSegments(); //if there are locked joints, the segment nr != the joint nr
    joint_map_.reset(new Eigen::VectorXi(n_chain_jnts));

    unsigned int k=0; //index of the unlocked joints
    for(unsigned int i=0; i<n_chain_sgmnts; i++)//iterate over all chain segments
    {
        for (unsigned int j=0; j<n_jnts; j++)//iterate over all controlled joints
        {
            if(chain_->getSegment(i).getJoint().getName() == joints_->at(j).getName())
            {
                (*joint_map_)(k)=j;//associate the unlocked chain joints with the corresponding controlled joints
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
void TaskObject::setChain(boost::shared_ptr<KDL::Chain> chain)
{
    chain_ = chain;
    updateJointMap();
}
//----------------------------------------------------
void TaskObject::setJoints(boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints)
{
    joints_ = joints;
    updateJointMap();
}
//----------------------------------------------------
std::string TaskObject::getFrame() const
{
    return chain_->segments.back().getName();
}
//----------------------------------------------------
boost::shared_ptr<KDL::Chain> TaskObject::getChain() const {return chain_;}
//----------------------------------------------------
boost::shared_ptr<std::vector< hardware_interface::JointHandle > > TaskObject::getJoints() const {return joints_;}
//----------------------------------------------------
void TaskObject::computeKinematics()
{
    //    //Iterate through all controlled joints and find the ones corresponding to the chain of the present task object
    //    Eigen::VectorXi active_joints = Eigen::VectorXi::Zero(joints.size()); //those joints which are active in the chain corresponding to the present task object
    //    unsigned int n_jnts = chain_->getNrOfJoints();
    //    KDL::JntArray q(n_jnts);
    //    q.data = Eigen::VectorXd::Zero(n_jnts);
    //    for(unsigned int i=0; i<n_jnts; i++)
    //    {
    //        std::cout<<"CONTROLLED JNTS VS CHAIN JNTS: "<<std::endl;
    //        if(chain_->segments[i].getJoint().getType()==KDL::Joint::None)
    //            continue;

    //        std::string jnt_name = chain_->segments[i].getJoint().getName();
    //        for(unsigned int j=0; j<joints.size(); j++)
    //        {
    //            std::cout<<joints.at(j).getName()<<" | "<<jnt_name<<std::endl;
    //            if(joints.at(j).getName() == jnt_name)
    //            {
    //                active_joints(j)=1;
    //                q.data(i)=joints[j].getPosition();
    //                break;
    //            }
    //        }

    //    }
    //    std::cout<<"ACTIVE JOINTS: "<<active_joints.transpose()<<std::endl<<std::endl;
    //    //========= DEBUG PRINT CHAIN ==========
    //    std::ostringstream out;
    //    printKDLChain(out, *chain_);
    //    std::cout<<"CHAIN:"<<std::endl;
    //    std::cout<<out.str()<<std::endl;
    //    //========= DEBUG PRINT CHAIN END ==========

    //    KDL::Frame pose;
    //    KDL::Jacobian jacobian(n_jnts);
    //    if(fk_solver_->JntToCart(q,pose) < 0)
    //        ROS_ERROR("Could not compute forward kinematics of task object with frame %s.",frame_.c_str());

    //    if(j_solver_->JntToJac(q,jacobian) < 0)
    //        ROS_ERROR("Could not compute jacobian of task object with frame %s.",frame_.c_str());

    //    KDLToEigen(pose, *pose_);
    //    KDLToEigen(jacobian, *jacobian_);

    //    std::cout<<"Task object w. frame: "<<frame_<<std::endl;
    //    std::cout<<"Pose translation: "<<std::endl<< pose_->translation().transpose()<<std::endl;
    //    std::cout<<"Pose rotation: "<<std::endl<< pose_->rotation()<<std::endl;
    //    std::cout<<"Jacobian: "<<std::endl<< *jacobian_<<std::endl;
    //    std::cout<<std::endl;

    //    exit(0);
}
//----------------------------------------------------
boost::shared_ptr<Eigen::Affine3d> TaskObject::getPose() const {return pose_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::MatrixXd> TaskObject::getJacobian() const {return jacobian_;}
//----------------------------------------------------
boost::shared_ptr<KDL::Jacobian> TaskObject::getChainJacobian() const {return chain_jacobian_;}
//----------------------------------------------------
void TaskObject::addGeometry(boost::shared_ptr<TaskGeometry> geometry)
{
    geometries_->push_back(geometry);
}
//----------------------------------------------------
boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > TaskObject::getGeometries() const {return geometries_;}
//----------------------------------------------------
} //end namespace hqp_controllers
