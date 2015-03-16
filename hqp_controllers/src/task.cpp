#include <hqp_controllers/task.h>
#include <hqp_controllers/utilities.h>
#include <typeinfo>

namespace hqp_controllers{
//---------------------------------------------------------
std::ostream& operator<<(std::ostream& str, Task const& task)
{
    str<<"TASK: "<<std::endl;
    str<<"id: "<<task.id_<<std::endl;
    str<<"type: "<<typeid(task).name()<<std::endl;
    str<<"task frame: "<<task.task_frame_<<std::endl;
    str<<"priority: "<<task.priority_<<std::endl;
    str<<"is equality task: "<<task.is_equality_task_<<std::endl;
    str<<"ds: "<<task.ds_<<std::endl;
    str<<"di: "<<task.di_<<std::endl;
    str<<"A_:"<<std::endl<<task.J_<<std::endl;
    str<<"E_:"<<std::endl<<task.E_<<std::endl;

    str<< *(task.t_dynamics_)<<std::endl;

    for(unsigned int i=0; i<task.t_links_.size(); i++)
        str<< *(task.t_links_.at(i));

    str<<std::endl;
}
//---------------------------------------------------------
Task::Task(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links): id_(id), priority_(priority), task_frame_(task_frame), is_equality_task_(is_equality_task), t_dynamics_(t_dynamics), t_links_(t_links), ds_(0.0), di_(1.0)
{
    ROS_ASSERT(t_dynamics_.get());
    ROS_ASSERT(t_dynamics_->getDimension() > 0);
    ROS_ASSERT(priority_ >= 1);

    ROS_ASSERT(t_dynamics_->getDimension() == 1); //1D state-space for now
}
//---------------------------------------------------------
void Task::setTaskVelocityDamping(double ds, double di)
{
    ROS_ASSERT((ds_ >= 0.0) && (di > 0.0));
    ds_ = ds;
    di_ = di;
}
//---------------------------------------------------------
//void Task::updateTaskFunctionDerivatives()
//{
//    ros::Time t = ros::Time::now();
//    if(t_start_)
//    {
//        t_prev_ = t;
//        t_start_ = false;
//    }
//    double dt = (t - t_prev_).toSec();
//    ROS_ASSERT(dt > -1e-8);

//    unsigned int d_dim = t_dynamics_->getDimension();

//    //Euler integration
//    Eigen::VectorXd e=E_.col(0); //save the task function values - those are not integrated
//    E_.leftCols(d_dim) = E_.leftCols(d_dim) + E_.rightCols(d_dim)*dt;
//    E_.col(0) = e;//put back the task function values

//    //compute new task function derivatives
//    Eigen::VectorXd dx(d_dim);
//    Eigen::VectorXd x(d_dim);
//    for(unsigned int i=0; i < E_.rows(); i++)
//    {
//        x = E_.row(i).head(d_dim).transpose();
//        t_dynamics_->getDX(dx,x);
//        E_.row(i).tail(d_dim) = dx.transpose();
//    }
//    t_prev_ = t;
//}
//---------------------------------------------------------
unsigned int Task::getId()const {return id_;}
//---------------------------------------------------------
void Task::setId(unsigned int id) {id_=id;}
//---------------------------------------------------------
void Task::setIsEqualityTask(bool is_equality_task){is_equality_task_ = is_equality_task;}
//---------------------------------------------------------
bool Task::getIsEqualityTask()const{return is_equality_task_;}
//---------------------------------------------------------
unsigned int Task::getPriority()const {return priority_;}
//---------------------------------------------------------
void Task::setPriority(unsigned int priority) {priority_= priority;}
//---------------------------------------------------------
Eigen::MatrixXd Task::getTaskJacobian()const{return J_;}
//---------------------------------------------------------
Eigen::VectorXd Task::getTaskFunction()const{return E_.col(0);}
//---------------------------------------------------------
Eigen::VectorXd Task::getTaskVelocity()const{return E_.col(1);}
//---------------------------------------------------------
////void Task::setTaskDynamics(boost::shared_ptr<TaskDynamics> t_dynamics)
////{
////    ROS_ASSERT(t_dynamics->getDimension() > 0);
////    t_dynamics_ = t_dynamics;
////}
////---------------------------------------------------------
//boost::shared_ptr<TaskDynamics> Task::getTaskDynamics()const{return t_dynamics_;}
//---------------------------------------------------------
XmlRpc::XmlRpcValue Task::taskMessageToXmlRpcValue(hqp_controllers_msgs::Task const& msg)
{
    XmlRpc::XmlRpcValue task;

    task["t_type"] = msg.t_type;
    task["priority"] = msg.priority;
    task["is_equality_task"] = msg.is_equality_task;
    task["task_frame"] = msg.task_frame;
    task["ds"] = msg.ds;
    task["di"] = msg.di;
    task["dynamics"]["d_type"] = msg.dynamics.d_type;
    for(unsigned int i=0; i<msg.dynamics.d_data.size();i++)
        task["dynamics"]["d_data"][i] = msg.dynamics.d_data[i];
    for(unsigned int i=0; i<msg.t_links.size();i++)
    {
        task["t_links"][i]["link_frame"] = msg.t_links[i].link_frame;
        for(unsigned int j=0; j<msg.t_links[i].geometries.size();j++)
        {
            task["t_links"][i]["geometries"][j]["g_type"] = msg.t_links[i].geometries[j].g_type;
            for(unsigned int k=0; k<msg.t_links[i].geometries[j].g_data.size();k++)
                task["t_links"][i]["geometries"][j]["g_data"][k]=msg.t_links[i].geometries[j].g_data[k];
        }
    }

    return task;
}
//---------------------------------------------------------
boost::shared_ptr<Task> Task::makeTask(unsigned int id, XmlRpc::XmlRpcValue& t_description, KDL::Tree const& k_tree, std::vector< hardware_interface::JointHandle > const& joints)
{
    ROS_ASSERT(t_description["t_type"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    TaskType t_type = static_cast<TaskType>((int)t_description["t_type"]);

    ROS_ASSERT(t_description["priority"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    int priority = (int)t_description["priority"];

    ROS_ASSERT(t_description["is_equality_task"].getType() == XmlRpc::XmlRpcValue::TypeInt); //TypeBoolean ain't working for some reason
    bool is_equality_task;
    if ((int)t_description["is_equality_task"] == 0)
        is_equality_task = false;
    else if ((int)t_description["is_equality_task"] == 1)
        is_equality_task = true;
    else
        ROS_BREAK();

    ROS_ASSERT(t_description["task_frame"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string task_frame = (std::string)t_description["task_frame"];

    ROS_ASSERT(t_description["ds"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    double ds = (double)t_description["ds"];

    ROS_ASSERT(t_description["di"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    double di = (double)t_description["di"];

    //make task dynamics
    ROS_ASSERT(t_description["dynamics"]["d_type"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    TaskDynamicsType d_type = static_cast<TaskDynamicsType>((int)t_description["dynamics"]["d_type"]);

    ROS_ASSERT(t_description["dynamics"]["d_data"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    Eigen::VectorXd d_data(t_description["dynamics"]["d_data"].size());
    for(unsigned int i=0; i<t_description["dynamics"]["d_data"].size(); i++)
        d_data(i) = t_description["dynamics"]["d_data"][i];

    boost::shared_ptr<TaskDynamics> t_dynamics = TaskDynamics::makeTaskDynamics(d_type, d_data);

    //make task links
    ROS_ASSERT(t_description["t_links"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<boost::shared_ptr<TaskLink> > t_links;
    for(unsigned int i=0; i<t_description["t_links"].size(); i++)
    {
        ROS_ASSERT(t_description["t_links"][i]["link_frame"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string link_frame = t_description["t_links"][i]["link_frame"];

        KDL::Chain chain;
        if(!k_tree.getChain(task_frame,link_frame, chain))
        {
            ROS_ERROR("Error in Task::makeTask(...): could not get kinematic chain from %s to %s.", task_frame.c_str(),link_frame.c_str());
            ROS_BREAK();
        }
        boost::shared_ptr<TaskLink> t_link(new TaskLink(task_frame, chain, joints));

        ROS_ASSERT(t_description["t_links"][i]["geometries"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(unsigned int j=0; j<t_description["t_links"][i]["geometries"].size(); j++)
        {
            TaskGeometryType g_type = static_cast<TaskGeometryType>((int)t_description["t_links"][i]["geometries"][j]["g_type"]);
            Eigen::VectorXd g_data(t_description["t_links"][i]["geometries"][j]["g_data"].size());
            for(unsigned int k=0; k<t_description["t_links"][i]["geometries"][j]["g_data"].size(); k++)
                g_data(k) = t_description["t_links"][i]["geometries"][j]["g_data"][k];

            boost::shared_ptr<TaskGeometry> geom = TaskGeometry::makeTaskGeometry(g_type, link_frame, task_frame, g_data);
            t_link->addGeometry(geom);
        }
        t_links.push_back(t_link);
    }

    //generate the corresponding task
    boost::shared_ptr<Task> task;

    if(t_type == PROJECTION)
        task.reset(new Projection(id, priority, task_frame, is_equality_task, t_dynamics, t_links));
    else if(t_type == ORIENTATION)
        task.reset(new Orientation(id, priority, task_frame, is_equality_task, t_dynamics, t_links));
    else if(t_type == JOINT_SETPOINT)
        task.reset(new JointSetpoint(id, priority, task_frame, is_equality_task, t_dynamics, t_links));
    else if(t_type == JOINT_LIMIT_AVOIDANCE)
        task.reset(new JointLimitAvoidance(id, priority, task_frame, is_equality_task, t_dynamics, t_links));
    //    else if(type == PARALLEL_LINES)
    //        task.reset(new ParallelLines(id, priority, sign, t_objs, t_dynamics));
    //    else if(type == ANGLE_LINES)
    //        task.reset(new AngleLines(id, priority, sign, t_objs, t_dynamics));
    //    else if(type == COPLANAR_LINES)
    //        task.reset(new CoplanarLines(id, priority, sign, t_objs, t_dynamics));
    //    else if(type == PROJECT_SPHERE_PLANE)
    //        task.reset(new ProjectSpherePlane(id, priority, sign, t_objs, t_dynamics));
    else
    {
        ROS_ERROR("Task type %d is invalid.", t_type);
        ROS_BREAK();
    }

    task->setTaskVelocityDamping(ds,di);
    return task;
}
//---------------------------------------------------------
void Task::computeTaskLinkKinematics()
{
    for(unsigned int i=0; i<t_links_.size(); i++)
        t_links_[i]->computeKinematics();
}
//---------------------------------------------------------
std::vector<boost::shared_ptr<TaskLink> > Task::getTaskLinks()const{return t_links_;}
//---------------------------------------------------------
Projection::Projection(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links) : Task(id, priority, task_frame, is_equality_task, t_dynamics, t_links)
{
    //    dim_ = t_objs_->at(1).getGeometries()->size();
    ROS_ASSERT(t_links_.size() == 2);
    ROS_ASSERT(t_links_.at(0).get() && t_links_.at(1).get()); //make sure the task links exist
    //make sure task geometries exist
    ROS_ASSERT((t_links_.at(0)->getGeometries().size() > 0) &&  (t_links_.at(1)->getGeometries().size() > 0));
    for (unsigned int i=0; i<t_links_.at(0)->getGeometries().size(); i++)
        ROS_ASSERT(t_links_.at(0)->getGeometries().at(i).get());
    for (unsigned int i=0; i<t_links_.at(1)->getGeometries().size(); i++)
        ROS_ASSERT(t_links_.at(1)->getGeometries().at(i).get());

}
//---------------------------------------------------------
void Projection::updateTask()
{
    //compute forward kinematics and jacobians
    computeTaskLinkKinematics();

    unsigned int n_jnts = t_links_[0]->getNumJoints();

    J_.resize(0, n_jnts);
    E_.resize(0, 2);
    unsigned int t_dim = 0;
    for (unsigned int i = 0; i<t_links_[0]->getGeometries().size(); i++)
        for (unsigned int j = 0; j<t_links_[1]->getGeometries().size(); j++)
        {
            ProjectableGeometry* geom1 = dynamic_cast<ProjectableGeometry*>(t_links_[0]->getGeometries().at(i).get());
            ProjectableGeometry* geom2 = dynamic_cast<ProjectableGeometry*>(t_links_[1]->getGeometries().at(j).get());
            ProjectionQuantities proj = geom1->project(* geom2);

            //std::cerr<<"Projection quantities: "<<std::endl<<proj<<std::endl;
            for (unsigned int k = 0; k<proj.d_.rows(); k++)
            {
                t_dim++;
                E_.conservativeResize(t_dim, Eigen::NoChange);
                J_.conservativeResize(t_dim, Eigen::NoChange);

                //in case of an inequality task, disregard the task component if it's outside the influence zone
                if((proj.d_(k) < -di_) && (!is_equality_task_))
                {
                    E_(t_dim - 1,0) = 0;
                    E_(t_dim - 1,1) = 0;
                    J_.bottomRows<1>().setZero();

                    continue;
                }

                //TASK FUNCTION
                Eigen::VectorXd x(1), dx(1);
                //apply task damping
                x(0) = (proj.d_(k) + ds_);
                t_dynamics_->getDX(dx, x);
                E_(t_dim - 1,0) = x(0);
                E_(t_dim - 1,1) = dx(0);

                //TASK JACOBIAN
                //get the jacobian of the current projection points
                Eigen::Vector3d delta_p1 = proj.P1_.col(k) - t_links_[0]->getLinkTransform().translation();
                Eigen::Vector3d delta_p2 = proj.P2_.col(k) - t_links_[1]->getLinkTransform().translation();

                //std::cerr<<"J1 with link frame: "<<t_links_[0]-> getLinkFrame()<<" and delta_p1: "<<delta_p1.transpose()<<std::endl<<t_links_[0]->getJacobian(delta_p1)<<std::endl;
                //std::cerr<<"J2 with link frame: "<<t_links_[1]-> getLinkFrame()<<" and delta_p2: "<<delta_p2.transpose() <<std::endl<<t_links_[1]->getJacobian(delta_p2)<<std::endl;

                Eigen::MatrixXd jac = t_links_[0]->getJacobian(delta_p1) - t_links_[1]->getJacobian(delta_p2) ;

                J_.bottomRows<1>() = proj.N_.col(k).transpose() * jac.topRows<3>();
            }

        }
    //compute task function derivatives
    //updateTaskFunctionDerivatives();
    //std::cerr<<"E_:"<<std::endl<<E_<<std::endl;
    //     std::cerr<<"J_:"<<std::endl<<J_<<std::endl;
    //      ROS_BREAK();
}
//---------------------------------------------------------
double Projection::getTaskProgress()const
{
    if (is_equality_task_)
        return pow(E_.col(0).norm(),2); //SSE of the task functions
    else
    {
        //for inequality projection tasks, a negative task function value per definiton indicates 0 sse
        double sse = 0;
        for (unsigned int i=0; i < E_.rows();i++)
            if(E_(i,0) > 0.0)
                sse+=pow(E_(i,0),2);

        return sse;
    }
}
//---------------------------------------------------------
Orientation::Orientation(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links) : Task(id, priority, task_frame, is_equality_task, t_dynamics, t_links)
{
    //    dim_ = t_objs_->at(1).getGeometries()->size();
    ROS_ASSERT(t_links_.size() == 2);
    ROS_ASSERT(t_links_.at(0).get() && t_links_.at(1).get()); //make sure the task links exist
    //make sure task geometries exist
    ROS_ASSERT((t_links_.at(0)->getGeometries().size() == 1) &&  (t_links_.at(1)->getGeometries().size() == 1));
    ROS_ASSERT(t_links_.at(0)->getGeometries().at(0).get());
    ROS_ASSERT(t_links_.at(1)->getGeometries().at(0).get());

    J_.resize(1, t_links_[0]->getNumJoints());
    J_.setZero();
    E_.resize(1, 2);
    E_.setZero();
}
//---------------------------------------------------------
void Orientation::updateTask()
{
    //compute forward kinematics and jacobians
    computeTaskLinkKinematics();

    OrientableGeometry* geom1 = dynamic_cast<OrientableGeometry*>(t_links_[0]->getGeometries().at(0).get());
    OrientableGeometry* geom2 = dynamic_cast<OrientableGeometry*>(t_links_[1]->getGeometries().at(0).get());
    OrientationQuantities ori = geom1->orient(* geom2);
    //std::cerr<<"Orientation quantities: "<<std::endl<<ori<<std::endl;

    //in case of an inequality task, disregard the task component if it's outside the influence zone
    if((ori.d_ < -di_) && (!is_equality_task_))
        return;

    //TASK FUNCTION
    Eigen::VectorXd x(1), dx(1);
    //apply task damping
    x(0) = (ori.d_ + ds_);
    t_dynamics_->getDX(dx, x);
    E_(0,0) = x(0);
    E_(0,1) = dx(0);

//    std::cerr<<"J1 with link frame: "<<t_links_[0]-> getLinkFrame()<<std::endl<<t_links_[0]->getJacobian()<<std::endl;
//    std::cerr<<"J2 with link frame: "<<t_links_[1]-> getLinkFrame()<<std::endl<<t_links_[1]->getJacobian()<<std::endl;

    Eigen::MatrixXd jac = t_links_[0]->getJacobian() - t_links_[1]->getJacobian(); //delta offset does not matter since we're only interested in the rotation part of the jacobian
    J_ = -ori.h_ * jac.bottomRows<3>();

//    std::cerr<<"E_:"<<std::endl<<E_<<std::endl;
//    std::cerr<<"J_:"<<std::endl<<J_<<std::endl;
}
//---------------------------------------------------------
double Orientation::getTaskProgress()const
{
    if (is_equality_task_)
        return pow(E_(0,0),2); //SSE of the task function
    else
    {
        //for inequality projection tasks, a negative task function value per definiton indicates 0 sse
        double sse = 0;
        if(E_(0,0) > 0.0)
            sse=pow(E_(0,0),2);

        return sse;
    }
}
//---------------------------------------------------------
JointSetpoint::JointSetpoint(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links) : Task(id, priority, task_frame, is_equality_task, t_dynamics, t_links)
{
    unsigned int n_links =  t_links_.size();
    ROS_ASSERT(n_links > 0);
    ROS_ASSERT(is_equality_task_); //For now, constrain this task to be an equality task

    for (unsigned int i=0; i<t_links_.size(); i++)
    {
        ROS_ASSERT(t_links_.at(i).get()); //make sure the task links exist
        ROS_ASSERT(t_links_.at(i)->getGeometries().size() == 1); //one joint geometry per task link
        ROS_ASSERT(t_links_.at(i)->getGeometries().at(0).get()); //make sure the geometry exist
        ROS_ASSERT( typeid(*t_links_.at(i)->getGeometries().at(0)) == typeid(JointPosition)); //make sure the geometry is a joint position
        ROS_ASSERT(t_links_.at(i)->getChain().segments.rbegin()->getJoint().getType() != KDL::Joint::None); //make sure its not a fixed joint
    }

    E_.resize(n_links, 2);
    E_.setZero();
    J_.resize(n_links, t_links_.at(0)->getJoints().size());
    J_.setZero();
}
//---------------------------------------------------------
void JointSetpoint::updateTask()
{
    //compute forward kinematics and jacobians
    computeTaskLinkKinematics();

    for (unsigned int i=0; i<t_links_.size(); i++)
    {
        //Get the setpoint
        double q_set = t_links_.at(i)->getGeometries().at(0)->getTaskData()(0);
        //Get the current joint value
        unsigned int jnt_index = t_links_.at(i)->getJointMap().tail<1>()(0);
        double q = t_links_.at(i)->getJoints().at(jnt_index).getPosition();
        // std::cerr<<"joint index: "<<jnt_index<<std::endl<<" q: "<<q<<" qset: "<<q_set<<std::endl;

        //CHECK FOR INFLUENCE ZONE HERE!!!
        Eigen::VectorXd x(1), dx(1);
        x(0) = q_set - q;
        t_dynamics_->getDX(dx, x);
        //APPLY TASK DAMPING HERE!
        E_(i, 0) = x(0);
        E_(i, 1) = dx(0);

        //Compute the task jacobian
        J_(i, jnt_index) = -1.0;
    }
    //    std::cerr<<"E_: "<<std::endl<<E_<<std::endl;
    //    std::cerr<<"J_: "<<std::endl<<J_<<std::endl;
}
//---------------------------------------------------------
double JointSetpoint::getTaskProgress()const
{
    //For now, the sse is only defined for equality tasks!
    return pow(E_.col(0).norm(), 2);
}
//---------------------------------------------------------
JointLimitAvoidance::JointLimitAvoidance(unsigned int id, unsigned int priority, std::string const& task_frame, bool is_equality_task, boost::shared_ptr<TaskDynamics> t_dynamics, std::vector<boost::shared_ptr<TaskLink> > const& t_links) : Task(id, priority, task_frame, is_equality_task, t_dynamics, t_links)
{
    unsigned int n_links =  t_links_.size();
    ROS_ASSERT(n_links > 0);
    ROS_ASSERT(!is_equality_task_); //avoidance has to be an inequality task by definition

    for (unsigned int i=0; i<t_links_.size(); i++)
    {
        ROS_ASSERT(t_links_.at(i).get()); //make sure the task links exist
        ROS_ASSERT(t_links_.at(i)->getGeometries().size() == 1); //one joint geometry per task link
        ROS_ASSERT(t_links_.at(i)->getGeometries().at(0).get()); //make sure the geometry exist
        ROS_ASSERT( typeid(*t_links_.at(i)->getGeometries().at(0)) == typeid(JointLimits)); //make sure the geometry is a joint position
        ROS_ASSERT(t_links_.at(i)->getChain().segments.rbegin()->getJoint().getType() != KDL::Joint::None); //make sure its not a fixed joint
    }
    //    ROS_ASSERT( (type == KDL::Joint::RotAxis) || (type == KDL::Joint::RotX) || (type == KDL::Joint::RotY) || (type == KDL::Joint::RotZ));

    E_.resize(2 * n_links, 2);
    E_.setZero();
    J_.resize(2 * n_links, t_links_.at(0)->getJoints().size());
    J_.setZero();
}
//---------------------------------------------------------
double JointLimitAvoidance::getTaskProgress()const
{
    double sse = 0.0;

    for(unsigned int i=0; i<t_links_.size(); i++)
    {
        if((E_(2*i,0) - ds_) < 0.0 )
            sse+=pow(E_(2*i,0) - ds_, 2);

        if((fabs(E_(2*i+1,0)) - ds_) < 0.0 )
            sse+=pow(fabs(E_(2*i+1,0)) - ds_, 2);

    }

    return sse;
}
//---------------------------------------------------------
void JointLimitAvoidance::updateTask()
{
    //compute forward kinematics and jacobians
    computeTaskLinkKinematics();

    for (unsigned int i=0; i<t_links_.size(); i++)
    {
        //Get the current joint value
        unsigned int jnt_index = t_links_.at(i)->getJointMap().tail<1>()(0);
        double q = t_links_.at(i)->getJoints().at(jnt_index).getPosition();
        double dq_max = t_links_.at(i)->getGeometries().at(0)->getTaskData()(0);
        double q_max = t_links_.at(i)->getGeometries().at(0)->getTaskData()(1);
        double q_min = t_links_.at(i)->getGeometries().at(0)->getTaskData()(2);

        //Task Jacobian
        J_(2*i, jnt_index) = 1.0;
        J_(2*i+1, jnt_index) = -1.0;

        //Task Function
        E_(2*i, 0) = q_max - q;
        E_(2*i+1, 0) = -q_max - q;

        //Task Velocity
        if(q > (q_max - di_))
            E_(2*i, 1) = dq_max*(E_(2*i, 0)  - ds_)/(di_ - ds_);
        else
            E_(2*i, 1) = dq_max;

        if(q < (q_min + di_))
            E_(2*i+1, 1) = dq_max*(E_(2*i+1, 0)  + ds_)/(ds_ - di_);
        else
            E_(2*i+1, 1) = dq_max;

    }

    // std::cerr<<"joint index: "<<jnt_index<<std::endl<<" q: "<<q<<std::endl;

}
//---------------------------------------------------------
//ParallelLines::ParallelLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
//{
//    type_ = PARALLEL_LINES;
//    dim_ = 3;

//    verifyTaskObjects();

//    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
//    A_->resize(dim_, n_jnts);
//    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
//    A_->setZero();
//    E_->setZero();

//    ROS_ASSERT(sign == "=");
//}
////---------------------------------------------------------
//double ParallelLines::getSSE()const
//{
//    return pow(E_->col(0).norm(), 2);
//}
////---------------------------------------------------------
//void ParallelLines::verifyTaskObjects()
//{
//    ROS_ASSERT(t_objs_->size() == 2); //need two Line geometries
//    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(1).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1); //need only one line geometry per task object
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->size() == 1);
//    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot());//make sure the task objects are w.r.t the same root frame
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == LINE);
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->at(0)->getType() == LINE);
//    //make sure that the first line is fixed in the environment for now
//    ROS_ASSERT(t_objs_->at(0).getChain()->getNrOfJoints() == 0);
//    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() > 0);
//}
////---------------------------------------------------------
//void ParallelLines::computeTask()
//{
//    //consider the first line attached to a static body, the second one to a movable one
//    // get the direction vectors of the two lines
//    Eigen::Vector3d u = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).tail<3>();
//    Eigen::Vector3d v = (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).tail<3>();

//    //std::cout<<"v: "<<v.transpose()<<std::endl;
//    //std::cout<<"u: "<<u.transpose()<<std::endl;
//    //std::cout<<"Jw: "<<std::endl<< t_objs_->at(1).getJacobian()->bottomRows<3>()<<std::endl;

//    Eigen::Vector3d v_neg = v*(-1);
//    //Compute the task jacobian
//    (*A_) = skewSymmetricMatrix(u) * skewSymmetricMatrix(v_neg) * t_objs_->at(1).getJacobian()->bottomRows<3>();

//    //Compute the task function:
//    E_->col(0) = u.cross(v);

//    updateTaskFunctionDerivatives();

//    //    std::cout<<"E_ :"<<std::endl<<(*E_)<<std::endl;
//    //    std::cout<<"A_ :"<<std::endl<<(*A_)<<std::endl;
//}
////---------------------------------------------------------
//AngleLines::AngleLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
//{
//    type_ = ANGLE_LINES;
//    dim_ = 1;

//    verifyTaskObjects();

//    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
//    A_->resize(dim_, n_jnts);
//    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
//    A_->setZero();
//    E_->setZero();
//}
////---------------------------------------------------------
//void AngleLines::verifyTaskObjects()
//{
//    ROS_ASSERT(t_objs_->size() == 2); //need a Cone and a Line geometry
//    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(1).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1); //need only one geometry per task object
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->size() == 1);
//    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot());//make sure the task objects are w.r.t the same root frame
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == CONE);
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->at(0)->getType() == LINE);
//    //make sure that the cone is fixed in the environment for now
//    ROS_ASSERT(t_objs_->at(0).getChain()->getNrOfJoints() == 0);
//    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() > 0);
//}
////---------------------------------------------------------
//double AngleLines::getSSE()const
//{
//    double e = 0;

//    if (sign_ == "=")
//        e = (*E_)(0,0);
//    else if(sign_ == ">=")
//    {
//        if((*E_)(0,0) < 0.0)
//            e = (*E_)(0,0);
//    }
//    else if(sign_ == "<=")
//    {
//        if((*E_)(0,0) > 0.0)
//            e = (*E_)(0,0);
//    }

//    return pow(e, 2);
//}
////---------------------------------------------------------
//void AngleLines::computeTask()
//{
//    //consider the first cone attached to a static body, the line to a movable one
//    Eigen::Vector3d u = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).segment(3,3); //cone direction
//    Eigen::Vector3d v = (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).tail<3>(); //line direction
//    double alpha = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).tail<1>()(0);

//    //    std::cout<<"v: "<<v.transpose()<<std::endl;
//    //    std::cout<<"u: "<<u.transpose()<<std::endl;
//    //    std::cout<<"alpha: "<<alpha<<std::endl;
//    //    std::cout<<"Jw: "<<std::endl<< t_objs_->at(1).getJacobian()->bottomRows<3>()<<std::endl;

//    //Compute the task jacobian
//    (*A_) = u.transpose() * skewSymmetricMatrix(v) * t_objs_->at(1).getJacobian()->bottomRows<3>();

//    //Compute the task function:
//    (*E_)(0,0) = cos(alpha) - u.transpose().dot(v);

//    updateTaskFunctionDerivatives();

//    //    std::cout<<"E_ :"<<std::endl<<(*E_)<<std::endl;
//    //    std::cout<<"A_ :"<<std::endl<<(*A_)<<std::endl;
//}
//---------------------------------------------------------
//CoplanarLines::CoplanarLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
//{
//    type_ = COPLANAR_LINES;
//    dim_ = 3;

//    verifyTaskObjects();

//    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
//    A_->resize(dim_, n_jnts);
//    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
//    A_->setZero();
//    E_->setZero();

//    ROS_ASSERT(sign == "=");
//}
////---------------------------------------------------------
//void CoplanarLines::verifyTaskObjects()
//{
//    ROS_ASSERT(t_objs_->size() == 2); //need two Line geometries
//    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(1).getGeometries().get()); //make sure a task geometry exists
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1); //need only one line geometry per task object
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->size() == 1);
//    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot());//make sure the task objects are w.r.t the same root frame
//    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == LINE);
//    ROS_ASSERT(t_objs_->at(1).getGeometries()->at(0)->getType() == LINE);
//    //make sure that the first line is fixed in the environment for now
//    ROS_ASSERT(t_objs_->at(0).getChain()->getNrOfJoints() == 0);
//    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() > 0);
//}
////---------------------------------------------------------
//double CoplanarLines::getSSE()const
//{
//    return pow(E_->col(0).norm(), 2);
//}
////---------------------------------------------------------
//void CoplanarLines::computeTask()
//{
//    //consider the first line attached to a static body, the second one to a movable one
//    Eigen::Vector3d u = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).tail<3>(); // get the direction vectors of the first one
//    Eigen::Vector3d a = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).head<3>(); // get the start vector of the first line

//    Eigen::Vector3d b = (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).head<3>();  // start of second line
//    Eigen::Vector3d c = b + (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).tail<3>();  //  another point on the second line

//    Eigen::Vector3d ab = a - b;
//    Eigen::Vector3d ac = a - c;

//    if((ab.norm() <= 1e-4) || (ac.norm() <= 1e-4))
//        ROS_WARN("CoplanarLines::computeTask(): ill conditioned task formulation!");

//    //ab.normalize(); ac.normalize();

//    Eigen::Vector3d delta_b =  b - t_objs_->at(1).getLinkTransform()->translation();
//    Eigen::Vector3d delta_c =  c - t_objs_->at(1).getLinkTransform()->translation();

//    boost::shared_ptr<Eigen::MatrixXd> jb = t_objs_->at(1).getJacobian(delta_b);
//    boost::shared_ptr<Eigen::MatrixXd> jc = t_objs_->at(1).getJacobian(delta_c);

//    //compute the task jacobian
//    (*A_) = skewSymmetricMatrix((-1*ac).cross(u))*skewSymmetricMatrix(u)*jb->topRows<3>() + skewSymmetricMatrix((-1*ab).cross(u))*skewSymmetricMatrix(u)*jc->topRows<3>();

//    //compute the task function
//    E_->col(0) = (u.cross(ab)).cross(u.cross(ac));

//    //    std::cout<<"u: "<<u.transpose()<<std::endl;
//    //    std::cout<<"a: "<<a.transpose()<<std::endl;
//    //    std::cout<<"b: "<<b.transpose()<<std::endl;
//    //    std::cout<<"c: "<<c.transpose()<<std::endl;
//    //    std::cout<<"ab: "<<ab.transpose()<<std::endl;
//    //    std::cout<<"ac: "<<ac.transpose()<<std::endl;
//    //    std::cout<<"delta_b: "<<delta_b.transpose()<<std::endl;
//    //    std::cout<<"delta_c: "<<delta_c.transpose()<<std::endl;

//    updateTaskFunctionDerivatives();

//    //       std::cout<<"E_ :"<<std::endl<<(*E_)<<std::endl;
//    //       std::cout<<"A_ :"<<std::endl<<(*A_)<<std::endl;
//}
//---------------------------------------------------------
} //end namespace hqp_controllers
