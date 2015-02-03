#include <hqp_controllers/task.h>
#include <hqp_controllers/utilities.h>

namespace hqp_controllers{
//---------------------------------------------------------
#define LIM_DT 0.1 ///< Limit for the time difference for state integration
//----------------------------------------------------
std::ostream& operator<<(std::ostream& str, Task const& task)
{
    str<<"TASK: "<<std::endl;
    str<<"id: "<<task.id_<<std::endl;
    str<<"type: "<<task.type_<<std::endl;
    str<<"priority: "<<task.priority_<<std::endl;
    str<<"sign: "<<task.sign_<<std::endl;
    str<<"dim: "<<task.dim_<<std::endl;
    str<<"A_:"<<std::endl<<(*task.A_)<<std::endl;
    str<<"E_:"<<std::endl<<(*task.E_)<<std::endl;
    str<<"t_prev_:"<<task.t_prev_.toSec()<<std::endl;
    str<<"Task objects:";
    for(unsigned int i=0; i<task.t_objs_->size(); i++)
        str<<task.t_objs_->at(i);

    str<<std::endl;
}
//---------------------------------------------------------
Task::Task(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics): type_(UNDEFINED_TASK), id_(id), priority_(priority), sign_(sign), t_objs_(t_objs), t_dynamics_(t_dynamics)
{
    ROS_ASSERT(t_objs_.get());
    ROS_ASSERT(t_dynamics_.get());
    ROS_ASSERT(t_dynamics_->getDimension() > 0);
    ROS_ASSERT( (sign_ == "=") || (sign_ == "<=") || (sign_ == ">=") );

    A_.reset(new Eigen::MatrixXd);
    E_.reset(new Eigen::MatrixXd);

    t_prev_ = ros::Time::now();
}
//---------------------------------------------------------
void Task::updateTaskFunctionDerivatives()
{
    ros::Time t = ros::Time::now();
    double dt = (t - t_prev_).toSec();
    ROS_ASSERT(dt > -1e-8);
    if(dt > LIM_DT)
    {
        ROS_WARN("In task id %d, Task::updateTaskFunctionDerivatives(...): sampling time dt=%f exeeds sampling time limit %f, setting dt=0.0.",id_,dt,LIM_DT);
        dt = 0.0;
    }
    unsigned int t_state_dim = t_dynamics_->getDimension();

    //Euler integration
    Eigen::VectorXd e=E_->col(0); //save the task function values - those are not integrated
    E_->leftCols(t_state_dim) = E_->leftCols(t_state_dim) + E_->rightCols(t_state_dim)*dt;
    E_->col(0) = e;//put back the task function values

    //compute new task function derivatives
    Eigen::VectorXd dx(t_state_dim);
    Eigen::VectorXd x(t_state_dim);
    for(unsigned int i=0; i < dim_; i++)
    {
        x = E_->row(i).head(t_state_dim).transpose();
        t_dynamics_->getDX(dx,x);
        E_->row(i).tail(t_state_dim) = dx.transpose();
    }
    t_prev_ = t;
}
//---------------------------------------------------------
unsigned int Task::getId()const {return id_;}
//---------------------------------------------------------
TaskType Task::getType()const {return type_;}
//---------------------------------------------------------
void Task::setId(unsigned int id) {id_=id;}
//---------------------------------------------------------
void Task::setSign(const std::string &sign){sign_ = sign;}
//---------------------------------------------------------
std::string Task::getSign()const{return sign_;}
unsigned int Task::getDimension()const{return dim_;}
//---------------------------------------------------------
unsigned int Task::getPriority()const {return priority_;}
//---------------------------------------------------------
void Task::setPriority(unsigned int priority) {priority_= priority;}
//---------------------------------------------------------
boost::shared_ptr<Eigen::MatrixXd> Task::getTaskJacobian()const{return A_;}
//---------------------------------------------------------
boost::shared_ptr<Eigen::VectorXd> Task::getTaskFunction()const
{
    return boost::shared_ptr<Eigen::VectorXd>(new Eigen::VectorXd(E_->col(0)));
}
//---------------------------------------------------------
boost::shared_ptr<Eigen::VectorXd> Task::getTaskVelocity()const
{
    return boost::shared_ptr<Eigen::VectorXd>(new Eigen::VectorXd(E_->col(1)));
}
//---------------------------------------------------------
boost::shared_ptr<std::vector<TaskObject> > Task::getTaskObjects()const{return t_objs_;}
//---------------------------------------------------------
//void Task::setTaskDynamics(boost::shared_ptr<TaskDynamics> t_dynamics)
//{
//    ROS_ASSERT(t_dynamics->getDimension() > 0);
//    t_dynamics_ = t_dynamics;
//}
//---------------------------------------------------------
boost::shared_ptr<TaskDynamics> Task::getTaskDynamics()const{return t_dynamics_;}
//---------------------------------------------------------
boost::shared_ptr<Task> Task::makeTask(unsigned int id, unsigned int priority, TaskType type, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics)
{
    boost::shared_ptr<Task> task;

    if(type == PROJECT_POINT_PLANE)
        task.reset(new ProjectPointPlane(id, priority, sign, t_objs, t_dynamics));
    else if(type == JOINT_SETPOINT)
        task.reset(new JointSetpoint(id, priority, sign, t_objs, t_dynamics));
    else if(type == JOINT_VELOCITY_LIMITS)
        task.reset(new JointVelocityLimits(id, priority, sign, t_objs, t_dynamics));
    else if(type == PARALLEL_LINES)
        task.reset(new ParallelLines(id, priority, sign, t_objs, t_dynamics));
    else if(type == ANGLE_LINES)
        task.reset(new AngleLines(id, priority, sign, t_objs, t_dynamics));
    else
    {
        ROS_ERROR("Task type %d is invalid.",type);
        ROS_BREAK();
    }
    return task;
}
//---------------------------------------------------------
ProjectPointPlane::ProjectPointPlane(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = PROJECT_POINT_PLANE;
    dim_ = t_objs_->at(1).getGeometries()->size();

    verifyTaskObjects();

    unsigned int n_jnts = t_objs_->at(1).getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();
}
//---------------------------------------------------------
void ProjectPointPlane::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_->size() == 2); //need one point and one set of planes
    ROS_ASSERT(t_objs_->at(0).getGeometries().get() && t_objs_->at(1).getGeometries().get()); //make sure the geometries exist
    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot()); //make sure the geometries associated with the task objects are formed in the same root frame
    //check that the task object geometries are valid - first one has to be a single point, second one a set of planes
    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1);
    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == POINT);
    ROS_ASSERT(dim_ > 0);
    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() == 0);//Make sure the planes are fixed in the environment for now
    for (unsigned int i=0; i<dim_;i++)
        ROS_ASSERT(t_objs_->at(1).getGeometries()->at(i)->getType() == PLANE);

}
//---------------------------------------------------------
//void ProjectPointPlane::setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs)
//{
//    t_objs_ = t_objs;
//    n_planes_ = t_objs_.second->getGeometries()->size();
//    verifyTaskObjects();

//    unsigned int n_jnts = t_objs_.first->getJacobian()->cols(); //number of controlled joints
//    A_->resize(n_planes_, n_jnts);
//    e_->resize(n_planes_);
//    de_->resize(n_planes_);
//    A_->setZero();
//    e_->setZero();
//    de_->setZero();
//}
//---------------------------------------------------------
void ProjectPointPlane::computeTask()
{
    //   std::cout<<"joints: ";
    //    for(unsigned int i = 0; i<t_objs_.first->getJoints()->size();i++)
    //        std::cout<<t_objs_.first->getJoints()->at(i).getPosition()<<" ";

    //Get the task point p(q) expressed in the task root frame
    Eigen::Vector3d p = (*(t_objs_->at(0).getGeometries()->at(0)->getRootData()));

    //Get the vector from the link frame origin to the task point expressed in the task object root frame
    Eigen::Vector3d delta_p = p - t_objs_->at(0).getLinkTransform()->translation();

    //Get the Jacobain w.r.t the task point p(q)
    boost::shared_ptr<Eigen::MatrixXd> jac = t_objs_->at(0).getJacobian(delta_p);

    //Compute the task function values and jacobians
    Eigen::VectorXd plane(4);
    for(unsigned int i=0; i<dim_;i++)
    {
        plane = (*(t_objs_->at(1).getGeometries()->at(i)->getRootData()));
        //task function values
        (*E_)(i,0) = plane.head<3>().transpose()*p-plane.tail<1>()(0);

        A_->row(i) = plane.head<3>().transpose() * jac->topRows<3>();
    }

    //    std::cout<<"NEW TASK TO COMPUTE"<<std::endl;
    //    std::cout<<std::endl<<"delta_p: "<<delta_p.transpose()<<std::endl;
    //    std::cout<<"p: "<<p.transpose()<<std::endl;
    //    std::cout<<"plane link: "<<(*(t_objs_.second->getGeometries()->at(0)->getLinkData())).transpose()<<std::endl;
    //    std::cout<<"plane root: "<<plane.transpose()<<std::endl;
    //    std::cout<<"pos jac:"<<std::endl<<jac->topRows<3>()<<std::endl;
    //    std::cout<<"A_:"<<std::endl<<(*A_)<<std::endl;

    //compute task function derivatives
    updateTaskFunctionDerivatives();

    //    std::cout<<GRB_DoubleParam_Cutoff<<std::endl;
}
//---------------------------------------------------------
JointSetpoint::JointSetpoint(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = JOINT_SETPOINT;
    dim_ = 1;

    verifyTaskObjects();

    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();

    unsigned int n_sgmnts =  t_objs_->at(0).getChain()->getNrOfSegments();
    std::string jnt_name = t_objs_->at(0).getChain()->getSegment(n_sgmnts - 1).getJoint().getName();
    KDL::Joint::JointType type = t_objs_->at(0).getChain()->getSegment(n_sgmnts - 1).getJoint().getType();
    //Translational joints are not allowed for now ...
    ROS_ASSERT( (type == KDL::Joint::RotAxis) || (type == KDL::Joint::RotX) || (type == KDL::Joint::RotY) || (type == KDL::Joint::RotZ));

    jnt_index_ = -1;
    for(unsigned int i=0; i<t_objs_->at(0).getJoints()->size(); i++)
        if(jnt_name == t_objs_->at(0).getJoints()->at(i).getName())
            jnt_index_ = i;

    ROS_ASSERT(jnt_index_ > -1); //make sure the joint was found
}
//---------------------------------------------------------
void JointSetpoint::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_->size() == 1); //need one Joint Setpoint
    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == JOINT_POSITION);
}
//---------------------------------------------------------
void JointSetpoint::computeTask()
{
    //Get the setpoint
    double q_set = (*t_objs_->at(0).getGeometries()->at(0)->getRootData())(0);

    //Get the current joint value
    double q = t_objs_->at(0).getJoints()->at(jnt_index_).getPosition();

    //Compute the task function
    (*E_)(0,0) = q_set - q;

    //Compute the task jacobian
    (*A_)(0,jnt_index_) = -1.0;

    //compute task function derivatives
    updateTaskFunctionDerivatives();

    //    std::cout<<"JointSetpoint: A_: "<<std::endl<<(*A_)<<std::endl;
    //    std::cout<<"JointSetpoint: E_: "<<std::endl<<(*E_)<<std::endl;
}
//---------------------------------------------------------
JointVelocityLimits::JointVelocityLimits(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = JOINT_VELOCITY_LIMITS;
    dim_ = 2;

    verifyTaskObjects();

    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();

    unsigned int n_sgmnts =  t_objs_->at(0).getChain()->getNrOfSegments();
    std::string jnt_name = t_objs_->at(0).getChain()->getSegment(n_sgmnts - 1).getJoint().getName();
    KDL::Joint::JointType type = t_objs_->at(0).getChain()->getSegment(n_sgmnts - 1).getJoint().getType();
    //Translational joints are not allowed for now ...
    ROS_ASSERT( (type == KDL::Joint::RotAxis) || (type == KDL::Joint::RotX) || (type == KDL::Joint::RotY) || (type == KDL::Joint::RotZ));

    jnt_index_ = -1;
    for(unsigned int i=0; i<t_objs_->at(0).getJoints()->size(); i++)
        if(jnt_name == t_objs_->at(0).getJoints()->at(i).getName())
            jnt_index_ = i;

    ROS_ASSERT(jnt_index_ > -1); //make sure the joint was found
    //Check the dynamics - for this task the gain of 1d linear dynamics is used as the value for the l1 norm of the joint velocity maximum
    ROS_ASSERT(t_dynamics_->getType() == LINEAR_DYNAMICS);
    ROS_ASSERT(t_dynamics_->getDimension() == 1);
    ROS_ASSERT((*static_cast<LinearTaskDynamics*>(t_dynamics_.get())->getDynamicsMatrix())(0,0) >= 0.0); //velocity maximum has to be positive
    //sign has to be smaller/equal for this task
    ROS_ASSERT(sign == "<=");
}
//---------------------------------------------------------
void JointVelocityLimits::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_->size() == 1); //need one Joint Limits geometry
    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == JOINT_LIMITS);
}
//---------------------------------------------------------
void JointVelocityLimits::computeTask()
{
    //Get the current joint value
    double q = t_objs_->at(0).getJoints()->at(jnt_index_).getPosition();

    //Get the joint limits
    Eigen::VectorXd limits = t_objs_->at(0).getGeometries()->at(0)->getRootData()->tail<6>();

    //Compute the task function
    if(q >= limits(5))
        (*E_)(0,0) = (q - limits(4))/(limits(5) - limits(4));
    else
        (*E_)(0,0) = 1.0;

    if(q <= limits(2))
        (*E_)(1,0) = (q - limits(1))/(limits(2) - limits(1));
    else
        (*E_)(1,0) = 1.0;

    //Compute the task jacobian
    (*A_)(0,jnt_index_) = 1.0;
    (*A_)(1,jnt_index_) = (-1.0);

    //compute task function derivatives
    updateTaskFunctionDerivatives();

    //std::cout<<"JointVelocityLimits: A_: "<<std::endl<<(*A_)<<std::endl;
    //std::cout<<"JointVelocityLimits: E_: "<<std::endl<<(*E_)<<std::endl;
}
//---------------------------------------------------------
ParallelLines::ParallelLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = PARALLEL_LINES;
    dim_ = 3;

    verifyTaskObjects();

    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();

    ROS_ASSERT(sign == "=");
}
//---------------------------------------------------------
void ParallelLines::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_->size() == 2); //need two Line geometries
    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(1).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1); //need only one line geometry per task object
    ROS_ASSERT(t_objs_->at(1).getGeometries()->size() == 1);
    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot());//make sure the task objects are w.r.t the same root frame
    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == LINE);
    ROS_ASSERT(t_objs_->at(1).getGeometries()->at(0)->getType() == LINE);
    //make sure that the first line is fixed in the environment for now
    ROS_ASSERT(t_objs_->at(0).getChain()->getNrOfJoints() == 0);
    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() > 0);
}
//---------------------------------------------------------
void ParallelLines::computeTask()
{
    //consider the first line attached to a static body, the second one to a movable one
    // get the direction vectors of the two lines
    Eigen::Vector3d u = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).tail<3>();
    Eigen::Vector3d v = (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).tail<3>();

    //std::cout<<"v: "<<v.transpose()<<std::endl;
    //std::cout<<"u: "<<u.transpose()<<std::endl;
    //std::cout<<"Jw: "<<std::endl<< t_objs_->at(1).getJacobian()->bottomRows<3>()<<std::endl;

    Eigen::Vector3d v_neg = v*(-1);
    //Compute the task jacobian
    (*A_) = skewSymmetricMatrix(u) * skewSymmetricMatrix(v_neg) * t_objs_->at(1).getJacobian()->bottomRows<3>();

    //Compute the task function:
    E_->col(0) = u.cross(v);

    updateTaskFunctionDerivatives();

    //    std::cout<<"E_ :"<<std::endl<<(*E_)<<std::endl;
    //    std::cout<<"A_ :"<<std::endl<<(*A_)<<std::endl;
}
//---------------------------------------------------------
AngleLines::AngleLines(unsigned int id, unsigned int priority, std::string const& sign, boost::shared_ptr<std::vector<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = ANGLE_LINES;
    dim_ = 1;

    verifyTaskObjects();

    unsigned int n_jnts = t_objs_->at(0).getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();
}
//---------------------------------------------------------
void AngleLines::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_->size() == 2); //need a Cone and a Line geometry
    ROS_ASSERT(t_objs_->at(0).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(1).getGeometries().get()); //make sure a task geometry exists
    ROS_ASSERT(t_objs_->at(0).getGeometries()->size() == 1); //need only one geometry per task object
    ROS_ASSERT(t_objs_->at(1).getGeometries()->size() == 1);
    ROS_ASSERT(t_objs_->at(0).getRoot() == t_objs_->at(1).getRoot());//make sure the task objects are w.r.t the same root frame
    ROS_ASSERT(t_objs_->at(0).getGeometries()->at(0)->getType() == CONE);
    ROS_ASSERT(t_objs_->at(1).getGeometries()->at(0)->getType() == LINE);
    //make sure that the cone is fixed in the environment for now
    ROS_ASSERT(t_objs_->at(0).getChain()->getNrOfJoints() == 0);
    ROS_ASSERT(t_objs_->at(1).getChain()->getNrOfJoints() > 0);
}
//---------------------------------------------------------
void AngleLines::computeTask()
{
    //consider the first cone attached to a static body, the line to a movable one
    Eigen::Vector3d u = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).segment(3,3); //cone direction
    Eigen::Vector3d v = (*t_objs_->at(1).getGeometries()->at(0)->getRootData()).tail<3>(); //line direction
    double alpha = (*t_objs_->at(0).getGeometries()->at(0)->getRootData()).tail<1>()(0);

    //    std::cout<<"v: "<<v.transpose()<<std::endl;
    //    std::cout<<"u: "<<u.transpose()<<std::endl;
    //    std::cout<<"alpha: "<<alpha<<std::endl;
    //    std::cout<<"Jw: "<<std::endl<< t_objs_->at(1).getJacobian()->bottomRows<3>()<<std::endl;

    //Compute the task jacobian
    (*A_) = u.transpose() * skewSymmetricMatrix(v) * t_objs_->at(1).getJacobian()->bottomRows<3>();

    //Compute the task function:
    (*E_)(0,0) = cos(alpha) - u.transpose().dot(v);

    updateTaskFunctionDerivatives();

    //    std::cout<<"E_ :"<<std::endl<<(*E_)<<std::endl;
    //    std::cout<<"A_ :"<<std::endl<<(*A_)<<std::endl;
}
//---------------------------------------------------------
} //end namespace hqp_controllers
