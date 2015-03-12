#include <hqp_controllers/task_geometry.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>
#include <math.h>
#include <typeinfo>

namespace hqp_controllers {
//------------------------------------------------------
std::ostream& operator<<(std::ostream& str, ProjectionQuantities const& proj)
{
    str<<"PROJECTION QUANTITIES"<<std::endl;
    str<<"P1:"<<std::endl<<proj.P1_<<std::endl;
    str<<"P2:"<<std::endl<<proj.P2_<<std::endl;
    str<<"N:"<<std::endl<<proj.N_<<std::endl;
    str<<"d: "<<proj.d_.transpose()<<std::endl;
    str<<std::endl;
}
//------------------------------------------------------
ProjectionQuantities::ProjectionQuantities(Eigen::Matrix3Xd const& P1, Eigen::Matrix3Xd const& P2, Eigen::Matrix3Xd const& N, Eigen::VectorXd const& d) : P1_(P1), P2_(P2), N_(N), d_(d){}
//------------------------------------------------------
std::ostream& operator<<(std::ostream& str, TaskGeometry const& geom)
{
    str<<"TASK GEOMETRY"<<std::endl;
    str<<"type: "<<typeid(geom).name()<<std::endl;
    str<<"link frame: "<<geom.link_frame_<<std::endl;
    str<<"task frame: "<<geom.task_frame_<<std::endl;
    str<<"link data: "<<geom.link_data_.transpose()<<std::endl;
    str<<"task data: "<<geom.task_data_.transpose()<<std::endl;
    str<<std::endl;
}
//------------------------------------------------------
TaskGeometry::TaskGeometry() : link_frame_(""), task_frame_(""){}
//------------------------------------------------------
TaskGeometry::TaskGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : link_frame_(link_frame), task_frame_(task_frame), link_data_(link_data){}
//------------------------------------------------------
std::string TaskGeometry::getLinkFrame() const {return link_frame_;}
//------------------------------------------------------
std::string TaskGeometry::getTaskFrame() const {return task_frame_;}
//------------------------------------------------------
Eigen::VectorXd TaskGeometry::getLinkData() const {return link_data_;}
//------------------------------------------------------
Eigen::VectorXd TaskGeometry::getTaskData() const {return task_data_;}
//------------------------------------------------------
boost::shared_ptr<TaskGeometry> TaskGeometry::makeTaskGeometry(TaskGeometryType type, std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    boost::shared_ptr<TaskGeometry> geom;

    if(type == POINT)
        geom.reset(new Point(link_frame, task_frame, link_data));
//    else if(type == LINE)
//        geom.reset(new Line(link, root, link_data));
    else if(type == PLANE)
        geom.reset(new Plane(link_frame, task_frame, link_data));
//    else if(type == FRAME)
//        geom.reset(new Frame(link, root, link_data));
//    else if(type == CAPSULE)
//        geom.reset(new Capsule(link, root, link_data));
//    else if(type == JOINT_POSITION)
//        geom.reset(new JointPosition(link, root, link_data));
//    else if(type == JOINT_LIMITS)
//        geom.reset(new JointLimits(link, root, link_data));
//    else if(type == CONE)
//        geom.reset(new Cone(link, root, link_data));
//    else if(type == CYLINDER)
//        geom.reset(new Cylinder(link, root, link_data));
//    else if(type == SPHERE)
//        geom.reset(new Sphere(link, root, link_data));
    else
    {
        ROS_ERROR("Task geometry type %d is invalid.",type);
        ROS_BREAK();
    }
    return geom;
}
//------------------------------------------------------------------------
ProjectableGeometry::ProjectableGeometry() : TaskGeometry(){}
//------------------------------------------------------------------------
ProjectableGeometry::ProjectableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : TaskGeometry(link_frame, task_frame, link_data){}
//------------------------------------------------------------------------
Point::Point() : ProjectableGeometry(){}
//------------------------------------------------------------------------
Point::Point(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
    ROS_ASSERT(link_data_.rows() == 3); //point is described by x/y/z coordinates
}
//------------------------------------------------------------------------
void Point::transformTaskData(Eigen::Affine3d const& T_l_t)
{
   task_data_ = T_l_t * link_data_.head<3>();
}
//------------------------------------------------------------------------
ProjectionQuantities Point::project(const ProjectableGeometry &geom)const
{
   ROS_ASSERT(task_frame_ == geom.getTaskFrame());
   return geom.projectOntoPoint(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoPoint(const Point &point)const
{

ProjectionQuantities proj;
proj.P1_.resize(Eigen::NoChange, 1);
proj.P2_.resize(Eigen::NoChange, 1);
proj.N_.resize(Eigen::NoChange, 1);
proj.d_.resize(1);

proj.P1_ << point.getTaskData();
proj.P2_ << task_data_;
proj.N_ << proj.P1_ - proj.P2_;
proj.d_(0) = proj.N_.norm()*(-1);
proj.N_ = proj.N_/proj.d_(0);

   return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoPlane(const Plane &plane)const
{
std::cout<<"ATTENZIONE: Point::projectOntoPlane(...) is not implemented yet!"<<std::endl;
ROS_BREAK();
}
//------------------------------------------------------------------------
void Point::addMarker(visualization_msgs::MarkerArray& markers)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = link_frame_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = markers.markers.size();
    marker.lifetime = ros::Duration();//(0.1);
    geometry_msgs::Point p;
    p.x=link_data_(0);
    p.y=link_data_(1);
    p.z=link_data_(2);
    marker.points.push_back(p);
    marker.scale.x = POINT_SCALE;
    marker.scale.y = POINT_SCALE;
    marker.scale.z = POINT_SCALE;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
//Line::Line() : TaskGeometry()
//{
//    type_ = LINE;
//    p_.reset(new Eigen::Vector3d);
//    v_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//Line::Line(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = LINE;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Line::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 6);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction
//    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
//    v_.reset(new Eigen::Vector3d(link_data.tail<3>()));
//    v_->normalize(); //normalize the direction vector just to make sure
//}
////------------------------------------------------------------------------
//void Line::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    //transformation which points x in the line direction
//    Eigen::Quaterniond q;
//    q.setFromTwoVectors(Eigen::Vector3d::UnitX() ,(*v_));

//    visualization_msgs::Marker m;
//    m.header.frame_id = link_;
//    m.header.stamp = ros::Time::now();
//    m.lifetime = ros::Duration(0.1);
//    m.type =  visualization_msgs::Marker::ARROW;
//    m.action = visualization_msgs::Marker::ADD;
//    m.id = markers.markers.size();
//    m.pose.position.x = (*p_)(0);
//    m.pose.position.y = (*p_)(1);
//    m.pose.position.z = (*p_)(2);
//    m.pose.orientation.x = q.x();
//    m.pose.orientation.y = q.y();
//    m.pose.orientation.z = q.z();
//    m.pose.orientation.w = q.w();
//    m.scale.x = LINE_SCALE;
//    m.scale.y = 0.05 * LINE_SCALE;
//    m.scale.z = 0.05 * LINE_SCALE;
//    m.color.r = 0.0;
//    m.color.g = 1.0;
//    m.color.b = 1.0;
//    m.color.a = 1.0;
//    markers.markers.push_back(m);
//}
////------------------------------------------------------------------------
//void Line::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

//    root_data_.reset(new Eigen::VectorXd(6));
//    //Express the line's reference point in the root frame
//    root_data_->head<3>() = (*trans_l_r_) * (*p_);
//    root_data_->tail<3>() = trans_l_r_->linear() * (*v_);
//}
//------------------------------------------------------------------------
//void Line::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Line::computeWitnessPoints(...) not implemented yet!");
//}


//------------------------------------------------------------------------
Plane::Plane() : ProjectableGeometry() {}
//------------------------------------------------------------------------
Plane::Plane(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
 ROS_ASSERT(link_data_.rows() == 4); //plane is described by unit normal n and offset d

 //normalize on the plane normal just to be sure ...
 link_data_ = link_data_/link_data_.head<3>().norm();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::project(const ProjectableGeometry &geom)const
{
   ROS_ASSERT(task_frame_ == geom.getTaskFrame());
   return geom.projectOntoPlane(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoPoint(const Point &point)const
{
std::cout<<"ATTENZIONE: Plane::projectOntoPoint() is not implemented yet!"<<std::endl;
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoPlane(const Plane &plane)const
{
    ROS_ERROR("Error in Plane::projectOntoPlane(...): Cannot project plane onto plane!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
void Plane::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Vector3d n(link_data_.head<3>());
    double d = link_data_(3);

    //transformation which points x in the plane normal direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitX() , n);

    visualization_msgs::Marker marker;

    //normal
    marker.header.frame_id = link_frame_;
    marker.header.stamp = ros::Time::now();
    marker.type =  visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);
    marker.id = markers.markers.size();
    marker.pose.position.x = n(0) * d;
    marker.pose.position.y = n(1) * d;
    marker.pose.position.z = n(2) * d;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = LINE_SCALE;
    marker.scale.y = 0.05 * LINE_SCALE;
    marker.scale.z = 0.05 * LINE_SCALE;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);

    //plane
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,n);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.id = markers.markers.size();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = PLANE_SCALE;
    marker.scale.y = PLANE_SCALE;
    marker.scale.z = 0.0001;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
void Plane::transformTaskData(Eigen::Affine3d const& T_l_t)
{
    //Express the plane in the task frame: nt = R_l_t * nl; dt=R_l_r*nl*trans_l_r*(nl*dl)
    task_data_.head<3>() = T_l_t.linear() * link_data_.head<3>();
    task_data_.tail<1>() = task_data_.head<3>().transpose() * (T_l_t * (link_data_.head<3>() * link_data_.tail<1>()));
}
//------------------------------------------------------------------------
////void Plane::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
////{
////    ROS_WARN("Plane::computeWitnessPoints(...) not implemented yet!");
////}
////------------------------------------------------------------------------
//Capsule::Capsule() : TaskGeometry(), r_(0.0)
//{
//    type_ = CAPSULE;
//    p_.reset(new Eigen::Vector3d);
//    t_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//Capsule::Capsule(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = CAPSULE;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Capsule::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 7);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    p_.reset(new Eigen::Vector3d(link_data.head<3>())); //the first 3 entries of link_data correspond to the capsule's start point
//    t_.reset(new Eigen::Vector3d(link_data.segment<3>(3))); //the next 3 entries of link_data correspond to the capsule's end point
//    r_ = link_data.tail<1>()(0);
//}
////------------------------------------------------------------------------
//void Capsule::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    visualization_msgs::Marker m;

//    //spheres
//    m.header.frame_id = link_;
//    m.header.stamp = ros::Time::now();
//    m.type =  visualization_msgs::Marker::SPHERE;
//    m.action = visualization_msgs::Marker::ADD;
//    m.lifetime = ros::Duration(0.1);
//    m.id = markers.markers.size();
//    m.pose.position.x = (*p_)(0);
//    m.pose.position.y = (*p_)(1);
//    m.pose.position.z = (*p_)(2);
//    m.pose.orientation.x = 0;
//    m.pose.orientation.y = 0;
//    m.pose.orientation.z = 0;
//    m.pose.orientation.w = 1;
//    m.scale.x = r_;
//    m.scale.y = r_;
//    m.scale.z = r_;
//    m.color.r = 1.0;
//    m.color.g = 0.85;
//    m.color.b = 0.0;
//    m.color.a = 1.0;
//    markers.markers.push_back(m);

//    m.id = markers.markers.size();
//    m.pose.position.x = (*t_)(0);
//    m.pose.position.y = (*t_)(1);
//    m.pose.position.z = (*t_)(2);
//    markers.markers.push_back(m);

//    //cylinder
//    //transformation which points z in the capsule's cylinder direction
//    Eigen::Quaterniond q;
//    Eigen::Vector3d v=(*t_)-(*p_);
//    ROS_ASSERT(v.norm() > 0.0); //spheres should be handled extra for now ...
//    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,v);
//    m.type =  visualization_msgs::Marker::CYLINDER;
//    m.id = markers.markers.size();
//    m.pose.position.x = (*p_)(0)+v(0)/2;
//    m.pose.position.y = (*p_)(1)+v(1)/2;
//    m.pose.position.z = (*p_)(2)+v(2)/2;
//    m.pose.orientation.x = q.x();
//    m.pose.orientation.y = q.y();
//    m.pose.orientation.z = q.z();
//    m.pose.orientation.w = q.w();
//    m.scale.z = v.norm();
//    markers.markers.push_back(m);
//}
////------------------------------------------------------------------------
//void Capsule::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

//    root_data_.reset(new Eigen::VectorXd(7));
//    //Express the capsules geometry in the root frame
//    root_data_->head<3>() = (*trans_l_r_) * (*p_);
//    root_data_->segment<3>(3) = (*trans_l_r_) * (*t_);
//    root_data_->tail<1>()(0) = r_;
//}
////------------------------------------------------------------------------
////void Capsule::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
////{
////    ROS_WARN("Capsule::computeWitnessPoints(...) not implemented yet!");
////}
////------------------------------------------------------------------------
//Frame::Frame() : TaskGeometry()
//{
//    type_ = FRAME;
//    trans_f_l_.reset(new Eigen::Affine3d);
//}
////------------------------------------------------------------------------
//Frame::Frame(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = FRAME;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Frame::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    Eigen::Quaterniond q(trans_f_l_->linear());
//    visualization_msgs::Marker e;

//    //e_x
//    e.header.frame_id = link_;
//    e.header.stamp = ros::Time::now();
//    e.type = visualization_msgs::Marker::ARROW;
//    e.action = visualization_msgs::Marker::ADD;
//    e.lifetime = ros::Duration(0.1);
//    e.id = markers.markers.size();
//    e.pose.position.x = trans_f_l_->translation()(0);
//    e.pose.position.y = trans_f_l_->translation()(1);
//    e.pose.position.z = trans_f_l_->translation()(2);
//    e.pose.orientation.x = q.x();
//    e.pose.orientation.y = q.y();
//    e.pose.orientation.z = q.z();
//    e.pose.orientation.w = q.w();
//    e.scale.x = LINE_SCALE;
//    e.scale.y = 0.05 * LINE_SCALE;
//    e.scale.z = 0.05 * LINE_SCALE;
//    e.color.r = 1.0;
//    e.color.g = 0.0;
//    e.color.b = 0.0;
//    e.color.a = 1.0;
//    markers.markers.push_back(e);

//    //e_y
//    Eigen::Quaterniond qy(0.5, 0.5, 0.5, 0.5); //rotates from x to y
//    qy=q*qy;
//    e.id = markers.markers.size();
//    e.pose.orientation.x = qy.x();
//    e.pose.orientation.y = qy.y();
//    e.pose.orientation.z = qy.z();
//    e.pose.orientation.w = qy.w();
//    e.color.r = 0.0;
//    e.color.g = 1.0;
//    e.color.b = 0.0;
//    e.color.a = 1.0;
//    markers.markers.push_back(e);

//    //e_z
//    Eigen::Quaterniond qz(0.5, -0.5, -0.5, -0.5); //rotates from x to z
//    qz=q*qz;
//    e.id = markers.markers.size();
//    e.pose.orientation.x = qz.x();
//    e.pose.orientation.y = qz.y();
//    e.pose.orientation.z = qz.z();
//    e.pose.orientation.w = qz.w();
//    e.color.r = 0.0;
//    e.color.g = 0.0;
//    e.color.b = 1.0;
//    e.color.a = 1.0;
//    markers.markers.push_back(e);
//}
////------------------------------------------------------------------------
//void Frame::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 6);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    Eigen::Vector3d transl = link_data.head<3>();
//    Eigen::Vector3d rpy = link_data.tail<3>();
//    Eigen::Matrix3d rot;
//    //create a x-y-z rotation matrix
//    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
//    trans_f_l_.reset(new Eigen::Affine3d);
//    trans_f_l_->translation() = transl;
//    trans_f_l_->linear() = rot;
//}
////------------------------------------------------------------------------
//void Frame::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));
//    Eigen::Affine3d trans_f_r( (*trans_l_r_) * (*trans_f_l_) ); //frame expressed in root

//    root_data_.reset(new Eigen::VectorXd(6));
//    root_data_->head<3>() = trans_f_r.translation();
//    root_data_->tail<3>() = trans_f_r.linear().eulerAngles(0, 1, 2);
//}
//------------------------------------------------------------------------
JointPosition::JointPosition() : TaskGeometry(){}
//------------------------------------------------------------------------
JointPosition::JointPosition(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : TaskGeometry(link_frame, task_frame, link_data)
{
 ROS_ASSERT(link_data_.rows() == 1); //joint position is described by one value only
}
//------------------------------------------------------------------------
void JointPosition::transformTaskData(const Eigen::Affine3d &T_l_t)
{
    task_data_ = link_data_; //transformation ain't changing the joint position
}
//------------------------------------------------------------------------
void JointPosition::addMarker(visualization_msgs::MarkerArray& markers)
{

ROS_ERROR("Error in JointPosition::addMarker(...): not implemented yet!;");
ROS_BREAK();

//    Eigen::Quaterniond q(trans_j_r_0_->linear());
//    visualization_msgs::Marker a;

//    //transformation which points x in z direction for plotting the arrow
//    Eigen::Quaterniond q_rot;
//    q_rot.setFromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
//    q_rot = q * q_rot;
//    //joint axis
//    a.header.frame_id = root_;
//    a.header.stamp = ros::Time::now();
//    a.type = visualization_msgs::Marker::ARROW;
//    a.action = visualization_msgs::Marker::ADD;
//    a.lifetime = ros::Duration(0.1);
//    a.id = markers.markers.size();
//    a.pose.position.x = trans_j_r_0_->translation()(0);
//    a.pose.position.y = trans_j_r_0_->translation()(1);
//    a.pose.position.z = trans_j_r_0_->translation()(2);
//    a.pose.orientation.x = q_rot.x();
//    a.pose.orientation.y = q_rot.y();
//    a.pose.orientation.z = q_rot.z();
//    a.pose.orientation.w = q_rot.w();
//    a.scale.x = LINE_SCALE;
//    a.scale.y = 0.05 * LINE_SCALE;
//    a.scale.z = 0.05 * LINE_SCALE;
//    a.color.r = 1.0;
//    a.color.g = 0.4;
//    a.color.b = 0.0;
//    a.color.a = 1.0;
//    markers.markers.push_back(a);

//    //joint position indication line
//    visualization_msgs::Marker l;
//    geometry_msgs::Point p;
//    l.header.frame_id = root_;
//    l.header.stamp = ros::Time::now();
//    l.type = visualization_msgs::Marker::LINE_LIST;
//    l.action = visualization_msgs::Marker::ADD;
//    l.lifetime = ros::Duration(0.1);
//    l.id = markers.markers.size();
//    l.pose.position.x = trans_j_r_0_->translation()(0);
//    l.pose.position.y = trans_j_r_0_->translation()(1);
//    l.pose.position.z = trans_j_r_0_->translation()(2);
//    l.pose.orientation.x = q.x();
//    l.pose.orientation.y = q.y();
//    l.pose.orientation.z = q.z();
//    l.pose.orientation.w = q.w();
//    l.scale.x = LINE_WIDTH;
//    l.color.r = 1.0;
//    l.color.g = 0.0;
//    l.color.b = 1.0;
//    l.color.a = 1.0;
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = p.x + LINE_SCALE * cos(q_pos_);
//    p.y = p.y + LINE_SCALE * sin(q_pos_);
//    p.z = 0;
//    l.points.push_back(p);
//    markers.markers.push_back(l);

//    //current joint position indication line
//    q = Eigen::Quaterniond(trans_j_l_->linear());
//    l = visualization_msgs::Marker();
//    l.header.stamp = ros::Time::now();
//    l.type = visualization_msgs::Marker::LINE_LIST;
//    l.action = visualization_msgs::Marker::ADD;
//    l.header.frame_id = link_;
//    l.id = markers.markers.size();
//    l.pose.position.x = trans_j_l_->translation()(0);
//    l.pose.position.y = trans_j_l_->translation()(1);
//    l.pose.position.z = trans_j_l_->translation()(2);
//    l.pose.orientation.x = q.x();
//    l.pose.orientation.y = q.y();
//    l.pose.orientation.z = q.z();
//    l.pose.orientation.w = q.w();
//    l.scale.x = 1.2 * LINE_WIDTH;
//    l.color.r = 0.0;
//    l.color.g = 1.0;
//    l.color.b = 1.0;
//    l.color.a = 1.0;
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = 0.8 * LINE_SCALE;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    markers.markers.push_back(l);
}
////------------------------------------------------------------------------
//JointLimits::JointLimits()
//{
//    type_ = JOINT_LIMITS;
//    trans_j_l_.reset(new Eigen::Affine3d);
//    trans_j_r_0_.reset(new Eigen::Affine3d);
//    lb_.reset(new Eigen::Vector3d);
//    ub_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//JointLimits::JointLimits(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = JOINT_LIMITS;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void JointLimits::setLinkData(Eigen::VectorXd const& link_data)
//{
//    //Joint limits is described by a frame expressed in the link with z pointing in the joint axis and zero angle pointing in x
//    //link_data.segment(6,6) is the initial transformation of the joint to the root
//    //link_data.tail<6>() are the upper and lower bounds
//    ROS_ASSERT(link_data.rows() == 18);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    Eigen::Matrix3d rot;
//    Eigen::Vector3d transl = link_data.head<3>();
//    Eigen::Vector3d rpy = link_data.segment(3,3);

//    //transform from the joint frame to the link frame
//    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
//    trans_j_l_.reset(new Eigen::Affine3d);
//    trans_j_l_->translation() = transl;
//    trans_j_l_->linear() = rot;

//    //transform from the joint frame at q==0 to the root frame
//    trans_j_r_0_.reset(new Eigen::Affine3d);
//    transl = link_data.segment(6,3);
//    rpy = link_data.segment(9,3);
//    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
//    trans_j_r_0_->translation() = transl;
//    trans_j_r_0_->linear() = rot;

//    //lower bounds q_min, q_mins and q_mini
//    lb_.reset(new Eigen::Vector3d(link_data.segment(12,3)));
//    //upper bounds q_max, q_maxs and q_maxi
//    ub_.reset(new Eigen::Vector3d(link_data.tail<3>()));

//    //Make sure the given limits are consistent
//    ROS_ASSERT((*lb_)(2) <= (*ub_)(2));
//    for(unsigned int i=0;  i<2; i++)
//    {
//        ROS_ASSERT((*lb_)(i+1) >= (*lb_)(i));
//        ROS_ASSERT((*ub_)(i+1) <= (*ub_)(i));
//    }
//}
////------------------------------------------------------------------------
//void JointLimits::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));
//    Eigen::Affine3d trans_j_r( (*trans_l_r_) * (*trans_j_l_) ); //joint frame expressed in root

//    root_data_.reset(new Eigen::VectorXd(18));
//    root_data_->head<3>() = trans_j_r.translation();
//    root_data_->segment(3,3) = trans_j_r.linear().eulerAngles(0, 1, 2);
//    root_data_->tail<12>() = link_data_->tail<12>(); //initial transform from joint to root and the limits don't change
//}
////------------------------------------------------------------------------
//void JointLimits::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    Eigen::Quaterniond q(trans_j_r_0_->linear());
//    visualization_msgs::Marker a;
//    //transformation which points x in z direction for plotting the arrow
//    Eigen::Quaterniond q_rot;
//    q_rot.setFromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
//    q_rot = q * q_rot;
//    //joint axis
//    a.header.frame_id = root_;
//    a.header.stamp = ros::Time::now();
//    a.type = visualization_msgs::Marker::ARROW;
//    a.action = visualization_msgs::Marker::ADD;
//    a.lifetime = ros::Duration(0.1);
//    a.id = markers.markers.size();
//    a.pose.position.x = trans_j_r_0_->translation()(0);
//    a.pose.position.y = trans_j_r_0_->translation()(1);
//    a.pose.position.z = trans_j_r_0_->translation()(2);
//    a.pose.orientation.x = q_rot.x();
//    a.pose.orientation.y = q_rot.y();
//    a.pose.orientation.z = q_rot.z();
//    a.pose.orientation.w = q_rot.w();
//    a.scale.x = LINE_SCALE;
//    a.scale.y = 0.05 * LINE_SCALE;
//    a.scale.z = 0.05 * LINE_SCALE;
//    a.color.r = 1.0;
//    a.color.g = 0.4;
//    a.color.b = 0.0;
//    a.color.a = 1.0;
//    markers.markers.push_back(a);

//    //lb, ub indication lines
//    visualization_msgs::Marker l;
//    geometry_msgs::Point p;
//    l.header.frame_id = root_;
//    l.header.stamp = ros::Time::now();
//    l.type = visualization_msgs::Marker::LINE_LIST;
//    l.action = visualization_msgs::Marker::ADD;
//    l.lifetime = ros::Duration(0.1);
//    l.id = markers.markers.size();
//    l.pose.position.x = trans_j_r_0_->translation()(0);
//    l.pose.position.y = trans_j_r_0_->translation()(1);
//    l.pose.position.z = trans_j_r_0_->translation()(2);
//    l.pose.orientation.x = q.x();
//    l.pose.orientation.y = q.y();
//    l.pose.orientation.z = q.z();
//    l.pose.orientation.w = q.w();
//    l.scale.x = LINE_WIDTH;
//    l.color.r = 1.0;
//    l.color.g = 0.0;
//    l.color.b = 0.0;
//    l.color.a = 0.5;
//    p.x =LINE_SCALE * cos((*lb_)(0));
//    p.y =LINE_SCALE * sin((*lb_)(0));
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    l.points.push_back(p);
//    p.x =LINE_SCALE * cos((*ub_)(0));
//    p.y =LINE_SCALE * sin((*ub_)(0));
//    p.z = 0;
//    l.points.push_back(p);
//    p.x =LINE_SCALE * cos((*lb_)(1));
//    p.y =LINE_SCALE * sin((*lb_)(1));
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    l.points.push_back(p);
//    p.x =LINE_SCALE * cos((*ub_)(1));
//    p.y =LINE_SCALE * sin((*ub_)(1));
//    p.z = 0;
//    l.points.push_back(p);
//    p.x =LINE_SCALE * cos((*lb_)(2));
//    p.y =LINE_SCALE * sin((*lb_)(2));
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    l.points.push_back(p);
//    p.x =LINE_SCALE * cos((*ub_)(2));
//    p.y =LINE_SCALE * sin((*ub_)(2));
//    p.z = 0;
//    l.points.push_back(p);

//    std_msgs::ColorRGBA color;
//    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
//    for(unsigned int i = 0; i<4; i++)
//        l.colors.push_back(color);

//    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
//    for(unsigned int i = 0; i<4; i++)
//        l.colors.push_back(color);

//    color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
//    for(unsigned int i = 0; i<4; i++)
//        l.colors.push_back(color);

//    markers.markers.push_back(l);

//    //current joint position indication line
//    q = Eigen::Quaterniond(trans_j_l_->linear());
//    l = visualization_msgs::Marker();
//    l.header.stamp = ros::Time::now();
//    l.type = visualization_msgs::Marker::LINE_LIST;
//    l.action = visualization_msgs::Marker::ADD;
//    l.lifetime = ros::Duration(0.1);
//    l.header.frame_id = link_;
//    l.id = markers.markers.size();
//    l.pose.position.x = trans_j_l_->translation()(0);
//    l.pose.position.y = trans_j_l_->translation()(1);
//    l.pose.position.z = trans_j_l_->translation()(2);
//    l.pose.orientation.x = q.x();
//    l.pose.orientation.y = q.y();
//    l.pose.orientation.z = q.z();
//    l.pose.orientation.w = q.w();
//    l.scale.x = 1.2 * LINE_WIDTH;
//    l.color.r = 0.0;
//    l.color.g = 1.0;
//    l.color.b = 1.0;
//    l.color.a = 1.0;
//    p.x = 0;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    p.x = 0.8 * LINE_SCALE;
//    p.y = 0;
//    p.z = 0;
//    l.points.push_back(p);
//    markers.markers.push_back(l);
//}
////------------------------------------------------------------------------
//Cone::Cone() : TaskGeometry(), alpha_(0.0)
//{
//    type_ = CONE;
//    p_.reset(new Eigen::Vector3d);
//    v_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//Cone::Cone(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = CONE;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Cone::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 7);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    //first 3 entries of link_data are the cone's reference point, the second 3 entries the direction, the last entry is the opening angle
//    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
//    v_.reset(new Eigen::Vector3d(link_data.segment(3,3)));
//    v_->normalize(); //normalize the direction vector just to make sure
//    alpha_ = link_data(6);

//    ROS_ASSERT((alpha_ <= 3.1416) && (alpha_ >= -3.1416)); //make sure the angle is between +/- PI
//}
////------------------------------------------------------------------------
//void Cone::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    visualization_msgs::Marker m;

//    //cylinder for the cone opening
//    //transformation which points z in the cone's direction
//    Eigen::Quaterniond q;
//    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,(*v_));
//    m.header.stamp = ros::Time::now();
//    m.header.frame_id = link_;
//    m.type =  visualization_msgs::Marker::CYLINDER;
//    m.lifetime = ros::Duration(0.1);
//    //    m.action = visualization_msgs::Marker::ADD;
//    m.id = markers.markers.size();
//    m.pose.position.x = (*p_)(0) + (*v_)(0) * cos(alpha_) * CONE_SCALE;
//    m.pose.position.y = (*p_)(1) + (*v_)(1) * cos(alpha_) * CONE_SCALE;
//    m.pose.position.z = (*p_)(2) + (*v_)(2) * cos(alpha_) * CONE_SCALE;
//    m.pose.orientation.x = q.x();
//    m.pose.orientation.y = q.y();
//    m.pose.orientation.z = q.z();
//    m.pose.orientation.w = q.w();
//    m.color.r = 1.0;
//    m.color.g = 0.0;
//    m.color.b = 1.0;
//    m.color.a = 0.8;
//    m.scale.z = LINE_WIDTH;
//    m.scale.x = 2 * CONE_SCALE * sin(alpha_);
//    m.scale.y = 2 * CONE_SCALE * sin(alpha_);
//    markers.markers.push_back(m);

//    //line for the cylinder axis
//    m.type =  visualization_msgs::Marker::LINE_LIST;
//    m.id = markers.markers.size();
//    m.pose.position.x = 0;
//    m.pose.position.y = 0;
//    m.pose.position.z = 0;
//    m.pose.orientation.x = 0;
//    m.pose.orientation.y = 0;
//    m.pose.orientation.z = 0;
//    m.pose.orientation.w = 1;
//    geometry_msgs::Point p;
//    p.x = (*p_)(0);
//    p.y = (*p_)(1);
//    p.z = (*p_)(2);
//    m.points.push_back(p);
//    p.x = (*p_)(0) + (*v_)(0) * cos(alpha_) * CONE_SCALE;
//    p.y = (*p_)(1) + (*v_)(1) * cos(alpha_) * CONE_SCALE;
//    p.z = (*p_)(2) + (*v_)(2) * cos(alpha_) * CONE_SCALE;
//    m.points.push_back(p);
//    m.scale.x =  LINE_WIDTH;
//    markers.markers.push_back(m);
//}
////------------------------------------------------------------------------
//void Cone::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

//    root_data_.reset(new Eigen::VectorXd(7));
//    //Express the Cone's reference point in the root frame
//    root_data_->head<3>() = (*trans_l_r_) * (*p_);
//    root_data_->segment(3,3) = trans_l_r_->linear() * (*v_);
//    (*root_data_)(6) = alpha_; //opening angle doesn't change
//}
////------------------------------------------------------------------------
//Cylinder::Cylinder() : TaskGeometry(), r_(0.0)
//{
//    type_ = CYLINDER;
//    p_.reset(new Eigen::Vector3d);
//    v_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//Cylinder::Cylinder(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = CYLINDER;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Cylinder::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 7);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction, the last one is the radius
//    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
//    v_.reset(new Eigen::Vector3d(link_data.segment(3,3)));
//    v_->normalize(); //normalize the direction vector just to make sure
//    r_ = link_data.tail<1>()(0);
//}
////------------------------------------------------------------------------
//void Cylinder::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    //transformation which points z in the cylinder direction
//    Eigen::Quaterniond q;
//    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,(*v_));

//    visualization_msgs::Marker m;
//    m.header.frame_id = link_;
//    m.header.stamp = ros::Time::now();
//    m.type =  visualization_msgs::Marker::CYLINDER;
//    m.action = visualization_msgs::Marker::ADD;
//    m.lifetime = ros::Duration(0.1);
//    m.id = markers.markers.size();
//    m.pose.position.x = (*p_)(0) + (*v_)(0) * 0.5 * LINE_SCALE;
//    m.pose.position.y = (*p_)(1) + (*v_)(1) * 0.5 * LINE_SCALE;
//    m.pose.position.z = (*p_)(2) + (*v_)(2) * 0.5 * LINE_SCALE;
//    m.pose.orientation.x = q.x();
//    m.pose.orientation.y = q.y();
//    m.pose.orientation.z = q.z();
//    m.pose.orientation.w = q.w();
//    m.scale.x = 2 * r_;
//    m.scale.y = 2 * r_;
//    m.scale.z = LINE_SCALE;
//    m.color.r = 0.0;
//    m.color.g = 1.0;
//    m.color.b = 1.0;
//    m.color.a = 0.8;
//    markers.markers.push_back(m);
//}
////------------------------------------------------------------------------
//void Cylinder::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

//    root_data_.reset(new Eigen::VectorXd(7));
//    //Express the line's reference point in the root frame
//    root_data_->head<3>() = (*trans_l_r_) * (*p_);
//    root_data_->segment(3,3) = trans_l_r_->linear() * (*v_);
//    (*root_data_)(6) = r_; //transform doesn't change the radius
//}
////------------------------------------------------------------------------
//Sphere::Sphere() : TaskGeometry(), r_(0.0)
//{
//    type_ = SPHERE;
//    p_.reset(new Eigen::Vector3d);
//}
////------------------------------------------------------------------------
//Sphere::Sphere(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
//{
//    type_ = SPHERE;
//    setLinkData(link_data);
//}
////------------------------------------------------------------------------
//void Sphere::setLinkData(Eigen::VectorXd const& link_data)
//{
//    ROS_ASSERT(link_data.rows() == 4);
//    link_data_.reset(new Eigen::VectorXd(link_data));

//    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
//    r_ = link_data.tail<1>()(0);

//    ROS_ASSERT(r_ > 0.0);
//}
////------------------------------------------------------------------------
//void Sphere::setLinkTransform(Eigen::Affine3d const& trans_l_r)
//{
//    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

//    //Express the Sphere in the root frame
//    root_data_.reset(new Eigen::VectorXd(4));
//    root_data_->head<3>() = (*trans_l_r_) * (*p_);
//    (*root_data_)(3) = r_; ///< radius doesn't change under transformation
//}
////------------------------------------------------------------------------
//void Sphere::addMarker(visualization_msgs::MarkerArray& markers)
//{
//    visualization_msgs::Marker m;

//    m.header.frame_id = link_;
//    m.header.stamp = ros::Time::now();
//    m.type = visualization_msgs::Marker::SPHERE;
//    m.action = visualization_msgs::Marker::ADD;
//    m.id = markers.markers.size();
//    m.lifetime = ros::Duration(0.1);
//    m.pose.position.x = (*p_)(0);
//    m.pose.position.y = (*p_)(1);
//    m.pose.position.z = (*p_)(2);
//    m.pose.orientation.x = 0.0;
//    m.pose.orientation.y = 0.0;
//    m.pose.orientation.z = 0.0;
//    m.pose.orientation.w = 1.0;
//    m.scale.x = 2 * r_;
//    m.scale.y = 2 * r_;
//    m.scale.z = 2 * r_;
//    m.color.r = 1.0;
//    m.color.g = 0.0;
//    m.color.b = 0.0;
//    m.color.a = 1.0;

//    markers.markers.push_back(m);
//}
//------------------------------------------------------------------------
} //end namespace hqp_controllers
