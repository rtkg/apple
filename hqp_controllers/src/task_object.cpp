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
TaskObject::TaskObject() : id_(0), frame_("")
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
TaskObject::TaskObject(unsigned int id, boost::shared_ptr<KDL::Chain> chain, boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints) : id_(id), chain_(chain), joints_(joints)
{    pose_.reset(new Eigen::Affine3d);
     geometries_.reset(new std::vector<boost::shared_ptr<TaskGeometry> >);
      fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(*chain_));
       j_solver_.reset(new KDL::ChainJntToJacSolver(*chain_));

        updateJointMap();

         jacobian_.reset(new Eigen::MatrixXd(6,joints_->size()));
          jacobian_->setZero();
           chain_jacobian_.reset(new KDL::Jacobian(joint_map_->rows()));
           frame_ = chain_->segments.back().getName();
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

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(*chain_));
    j_solver_.reset(new KDL::ChainJntToJacSolver(*chain_));

    chain_jacobian_->resize(joint_map_->rows());
      frame_ = chain_->segments.back().getName();
}
//----------------------------------------------------
void TaskObject::setJoints(boost::shared_ptr<std::vector< hardware_interface::JointHandle > > joints)
{
    joints_ = joints;
    updateJointMap();

    jacobian_->resize(6,joints_->size());
    jacobian_->setZero();
}
//----------------------------------------------------
void TaskObject::setId(unsigned int id){id_ = id;}
//----------------------------------------------------
unsigned int TaskObject::getId()const {return id_;}
//----------------------------------------------------
std::string TaskObject::getFrame() const{return frame_;}
//----------------------------------------------------
boost::shared_ptr<KDL::Chain> TaskObject::getChain() const {return chain_;}
//----------------------------------------------------
boost::shared_ptr<std::vector< hardware_interface::JointHandle > > TaskObject::getJoints() const {return joints_;}
//----------------------------------------------------
void TaskObject::computeKinematics()
{
    //read the chain joint positions from the controlled joints
    unsigned int n_jnts = joint_map_->rows();
    KDL::JntArray q(n_jnts);
    for(unsigned int i=0; i<n_jnts; i++)
        q.data(i) = joints_->at((*joint_map_)(i)).getPosition();

    //compute the chain jacobian
    KDL::Frame pose;
    KDL::Jacobian jacobian(n_jnts);
    if(fk_solver_->JntToCart(q,pose) < 0)
        ROS_ERROR("Could not compute forward kinematics of task object with frame %s.",frame_.c_str());

    if(j_solver_->JntToJac(q, *chain_jacobian_) < 0)
        ROS_ERROR("Could not compute jacobian of task object with frame %s.",frame_.c_str());

    KDLToEigen(pose, *pose_);

    //map the chain jacobian to the controlled joint jacobian
    jacobian_->setZero();
    for(unsigned int i=0; i<n_jnts; i++)
        jacobian_->col((*joint_map_)(i)) = chain_jacobian_->data.col(i);


    std::cout<<"number of joints: "<<n_jnts<<std::endl;
    std::cout<<"Task object w. frame: "<<frame_<<std::endl;
    std::cout<<"Pose translation: "<<std::endl<< pose_->translation().transpose()<<std::endl;
    std::cout<<"Pose rotation: "<<std::endl<< pose_->rotation()<<std::endl;
    std::cout<<"Chain jacobian: "<<std::endl<<chain_jacobian_->data<<std::endl;
    std::cout<<"Jacobian: "<<std::endl<< *jacobian_<<std::endl;
    std::cout<<std::endl;

    exit(0);

    //    //========= DEBUG PRINT CHAIN ==========
    //    std::ostringstream out;
    //    printKDLChain(out, *chain_);
    //    std::cout<<"CHAIN:"<<std::endl;
    //    std::cout<<out.str()<<std::endl;
    //    //========= DEBUG PRINT CHAIN END ==========

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
