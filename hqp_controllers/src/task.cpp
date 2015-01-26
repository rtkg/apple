#include <hqp_controllers/task.h>

namespace hqp_controllers{
//---------------------------------------------------------
#define LIM_DT 0.1 ///< Limit for the time difference for state integration
//---------------------------------------------------------
Task::Task(unsigned int id, unsigned int priority, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : type_(UNDEFINED_TASK), id_(id), priority_(priority), sign_(sign), t_objs_(t_objs), t_dynamics_(t_dynamics)
{
    ROS_ASSERT(t_dynamics_.get());
    ROS_ASSERT(t_dynamics_->getDimension() > 0);
    ROS_ASSERT( (sign_ == "=") || (sign_ == "<=") || (sign_ == ">=") );

    A_.reset(new Eigen::MatrixXd);
    E_.reset(new Eigen::MatrixXd);

    t_prev_ = ros::Time::now();
}
//---------------------------------------------------------
void Task::updateTaskFunctionMatrix()
{
    ros::Time t = ros::Time::now();
    double dt = (t_prev_ - t).toSec();
    ROS_ASSERT(dt < LIM_DT);
    std::cout<<"dt: "<<dt<<std::endl;

    unsigned int t_state_dim = t_dynamics_->getDimension();
    //Euler integration
    for(unsigned int i=0; i < t_state_dim - 1; i++)
        E_->col(t_state_dim - i - 1) = E_->col(t_state_dim - i - 1) + E_->col(t_state_dim - i) * dt;

    //compute new task function derivatives
    Eigen::VectorXd dx(t_state_dim);
    Eigen::VectorXd x(t_state_dim);
    for(unsigned int i=0; i < t_state_dim - 1; i++)
    {
        x = E_->row(i).head(t_state_dim).transpose();
        t_dynamics_->getDX(dx,x);
        E_->row(i).tail(t_state_dim) = dx.transpose();
    }
    t_prev_ = t;
}
//---------------------------------------------------------
unsigned int Task::getId() {return id_;}
//---------------------------------------------------------
TaskType Task::getType() {return type_;}
//---------------------------------------------------------
void Task::setId(unsigned int id) {id_=id;}
//---------------------------------------------------------
void Task::setSign(const std::string &sign){sign_ = sign;}
//---------------------------------------------------------
std::string Task::getSign()const{return sign_;}
//---------------------------------------------------------
unsigned int Task::getPriority() {return priority_;}
//---------------------------------------------------------
void Task::setPriority(unsigned int priority) {priority_= priority;}
//---------------------------------------------------------
boost::shared_ptr<Eigen::MatrixXd> Task::getTaskJacobian()const{return A_;}
//---------------------------------------------------------
void Task::getTaskFunction(Eigen::VectorXd& e)const{ e = E_->col(0);}
//---------------------------------------------------------
//boost::shared_ptr<Eigen::VectorXd> Task::getTaskVelocity()const{return de_;}
//---------------------------------------------------------
std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > Task::getTaskObjects()const{return t_objs_;}
//---------------------------------------------------------
//void Task::setTaskDynamics(boost::shared_ptr<TaskDynamics> t_dynamics)
//{
//    ROS_ASSERT(t_dynamics->getDimension() > 0);
//    t_dynamics_ = t_dynamics;
//}
//---------------------------------------------------------
boost::shared_ptr<TaskDynamics> Task::getTaskDynamics()const{return t_dynamics_;}
//---------------------------------------------------------
boost::shared_ptr<Task>  makeTask(unsigned int id, unsigned int priority, TaskType type, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics)
{
    boost::shared_ptr<Task> task;

    if(type == POINT_IN_HALFSPACE)
        task.reset(new PointInHalfspace(id,priority, sign,t_objs,t_dynamics));
    // else if(type == LINE)
    // geom.reset(new Line(link, root, link_data));
    else
    {
        ROS_ERROR("Task type %d is invalid.",type);
        ROS_BREAK();
    }
    return task;
}
//---------------------------------------------------------
PointInHalfspace::PointInHalfspace(unsigned int id, unsigned int priority, std::string const& sign, std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs, boost::shared_ptr<TaskDynamics> t_dynamics) : Task(id, priority, sign, t_objs, t_dynamics)
{
    type_ = POINT_IN_HALFSPACE;
    dim_ = t_objs_.second->getGeometries()->size();
    verifyTaskObjects();

    unsigned int n_jnts = t_objs_.first->getJacobian()->cols(); //number of controlled joints
    A_->resize(dim_, n_jnts);
    E_->resize(dim_,t_dynamics_->getDimension()+1); //needs to be one higher to containt state plus derivatives in the matrix rows
    A_->setZero();
    E_->setZero();
}
//---------------------------------------------------------
void PointInHalfspace::verifyTaskObjects()
{
    ROS_ASSERT(t_objs_.first && t_objs_.second);
    ROS_ASSERT(t_objs_.first->getRoot() == t_objs_.second->getRoot()); //make sure the geometries associated with the task objects are formed in the same root frame
    //check that the task object geometries are valid - first one has to be a single point, second one a set of planes
    ROS_ASSERT(t_objs_.first->getGeometries()->size() == 1);
    ROS_ASSERT(t_objs_.first->getGeometries()->at(0)->getType() == POINT);

    ROS_ASSERT(dim_ > 0);
    ROS_ASSERT(t_objs_.second->getChain()->getNrOfJoints() == 0);//Make sure the plane is fixed in the environment for now
    for (unsigned int i=0; i+dim_;i++)
        ROS_ASSERT(t_objs_.second->getGeometries()->at(i)->getType() == PLANE);
}
//---------------------------------------------------------
//void PointInHalfspace::setTaskObjects(std::pair<boost::shared_ptr<TaskObject>, boost::shared_ptr<TaskObject> > t_objs)
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
void PointInHalfspace::computeTask()
{
    //Get the vector from the link frame origin to the task point expressed in the task object root frame
    Eigen::Vector3d delta_p = (*(t_objs_.first->getGeometries()->at(0)->getRootData()));

    //Get the task point p(q) expressed in the task root frame
    Eigen::Vector3d p = t_objs_.first->getLinkTransform()->translation()+delta_p;

    //Get the Jacobain w.r.t the task point p(q)
    Eigen::MatrixXd Jp;
//    t_objs_.first->changeJacRefPoint(Jp,delta_p);

    //Compute the task function values and jacobians
        Eigen::VectorXd plane(4);
    for(unsigned int i=0; i<dim_;i++)
    {
        plane = (*(t_objs_.first->getGeometries()->at(0)->getRootData()));
        //task function values
        (*E_)(i,0) = plane.head<3>().transpose()*p-plane.tail<1>()(0);

       A_->row(i) = plane.head<3>().transpose() * Jp;
    }

    //compute task function derivatives
    updateTaskFunctionMatrix();
}
//---------------------------------------------------------
} //end namespace hqp_controllers
