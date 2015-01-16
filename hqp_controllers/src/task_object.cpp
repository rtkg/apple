#include <hqp_controllers/task_object.h>
#include <hqp_controllers/conversions.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace hqp_controllers {

//----------------------------------------------------
TaskObject::TaskObject() : frame_(""), priority_(0)
{
    geometries_.reset(new std::vector<boost::shared_ptr<TaskGeometry> >);

    pose_.reset(new Eigen::Affine3d);
    jacobian_.reset(new Eigen::MatrixXd);
    chain_.reset(new KDL::Chain);
}
//----------------------------------------------------
TaskObject::TaskObject(std::string frame, unsigned int priority, boost::shared_ptr<KDL::Chain> chain) : frame_(frame), priority_(priority)
{
    geometries_.reset(new std::vector<boost::shared_ptr<TaskGeometry> >);
    chain_=chain;

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(*chain_));
    j_solver_.reset(new KDL::ChainJntToJacSolver(*chain_));

    pose_.reset(new Eigen::Affine3d);
    jacobian_.reset(new Eigen::MatrixXd);

    //    for(unsigned int i=0; i<chain.getNrOfSegments();i++)
    //    {
    //        KDL::Segment seg = chain.getSegment(i);
    //        Eigen::Affine3d ttip;
    //        KDLToEigen(seg.getJoint().ge,ttip);
    //        std::cout<<"name: "<<seg.getName()<<std::endl;
    //        std::cout<<"joint: "<<seg.getJoint().getName()<<std::endl;
    //        std::cout<<"ttip translation: "<<ttip.translation().transpose()<<std::endl;
    //        std::cout<<"ttip rotation: "<<ttip.linear()<<std::endl;
    //    }

    //    std::cout<<"chain jnt nr: "<<chain.getNrOfJoints()<<std::endl;
    //    std::cout<<"chain seg nr: "<<chain.getNrOfSegments()<<std::endl;
}
//----------------------------------------------------
void TaskObject::setFrame(std::string frame) {frame_ = frame;}
//----------------------------------------------------
std::string TaskObject::getFrame() const {return frame_;}
//----------------------------------------------------
void TaskObject::computeKinematics(const std::vector< hardware_interface::JointHandle >& joints)
{
    //Iterate through all controlled joints and find the ones corresponding to the chain of the present task object
    KDL::JntArray q(chain_->getNrOfJoints());
    for(unsigned int i=0; i<chain_->getNrOfJoints(); i++)
    {
        std::string jnt_name = chain_->segments[i].getJoint().getName();
        unsigned int j=0;
        while(1)
        {
            std::cout<<jnt_name<<" vs. "<<joints.at(j).getName()<<std::endl;
            if(joints.at(j).getName() == jnt_name)
            {
                q.data(i)=joints[j].getPosition();
                break;
            }
            j++;
        }
    }

    KDL::Frame pose;
    KDL::Jacobian jacobian;
    if(fk_solver_->JntToCart(q,pose) < 0)
        ROS_ERROR("Could not compute forward kinematics of task object with frame %s.",frame_.c_str());

    if(j_solver_->JntToJac(q,jacobian) < 0)
        ROS_ERROR("Could not compute jacobian of task object with frame %s.",frame_.c_str());

    KDLToEigen(pose, *pose_);
    KDLToEigen(jacobian, *jacobian_);

      std::cout<<"Task object w. frame: "<<frame_<<std::endl;
        std::cout<<"Pose translation: "<<std::endl<< pose_->translation().transpose()<<std::endl;
     std::cout<<"Pose rotation: "<<std::endl<< pose_->rotation()<<std::endl;
        std::cout<<"Jacobian: "<<std::endl<< *jacobian_<<std::endl;
      std::cout<<std::endl;
}
//----------------------------------------------------
boost::shared_ptr<Eigen::Affine3d> TaskObject::getPose() const {return pose_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::MatrixXd> TaskObject::getJacobian() const {return jacobian_;}
//----------------------------------------------------
void TaskObject::addGeometry(boost::shared_ptr<TaskGeometry> geometry)
{
    geometries_->push_back(geometry);
}
//----------------------------------------------------
boost::shared_ptr<std::vector<boost::shared_ptr<TaskGeometry> > > TaskObject::getGeometries() const {return geometries_;}
//----------------------------------------------------
unsigned int TaskObject::getPriority()const{return priority_;}
//----------------------------------------------------
void TaskObject::setPriority(unsigned int priority) {priority_ = priority;}
//----------------------------------------------------
} //end namespace hqp_controllers
