#include <hqp_controllers/task_geometry.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>
#include <math.h>

namespace hqp_controllers {


//------------------------------------------------------
std::ostream& operator<<(std::ostream& str, TaskGeometry const& geom)
{
    str<<"TASK GEOMETRY"<<std::endl;
    str<<"type: "<<geom.type_<<std::endl;
    str<<"link: "<<geom.link_<<std::endl;
    str<<"root: "<<geom.root_<<std::endl;
    str<<"link data: "<<geom.link_data_->transpose()<<std::endl;
    str<<"root data: "<<geom.root_data_->transpose()<<std::endl;
    str<<"link transform translation:"<<std::endl<<geom.trans_l_r_->translation().transpose()<<std::endl;
    str<<"link transform rotation:"<<std::endl<<geom.trans_l_r_->linear()<<std::endl;
    str<<std::endl;
}
//------------------------------------------------------
TaskGeometry::TaskGeometry() : type_(BASIC_GEOMETRY), link_(""), root_("")
{
    link_data_.reset(new Eigen::VectorXd);
    root_data_.reset(new Eigen::VectorXd);
    trans_l_r_.reset(new Eigen::Affine3d);
    trans_l_r_->setIdentity();
}
//------------------------------------------------------
TaskGeometry::TaskGeometry(std::string const& link, std::string const& root) : link_(link), root_(root)
{
    type_ = BASIC_GEOMETRY;
    link_data_.reset(new Eigen::VectorXd);
    root_data_.reset(new Eigen::VectorXd);
    trans_l_r_.reset(new Eigen::Affine3d);
    trans_l_r_->setIdentity();
}
//------------------------------------------------------
void TaskGeometry::setLink(std::string const& link){link_ = link;}
//------------------------------------------------------
void TaskGeometry::setRoot(std::string const& root){root_ = root;}
//------------------------------------------------------
std::string TaskGeometry::getLink() const {return link_;}
//------------------------------------------------------
std::string TaskGeometry::getRoot() const {return root_;}
//------------------------------------------------------
TaskGeometryType TaskGeometry::getType() const {return type_;}
//------------------------------------------------------
boost::shared_ptr<Eigen::VectorXd> TaskGeometry::getLinkData() const {return link_data_;}
//------------------------------------------------------
boost::shared_ptr<Eigen::VectorXd> TaskGeometry::getRootData() const {return root_data_;}
//------------------------------------------------------
boost::shared_ptr<Eigen::Affine3d> TaskGeometry::getLinkTransform() const {return trans_l_r_;}
//------------------------------------------------------
boost::shared_ptr<TaskGeometry>  TaskGeometry::makeTaskGeometry(TaskGeometryType type, std::string const& link, std::string const& root, Eigen::VectorXd const& link_data)
{
    boost::shared_ptr<TaskGeometry> geom;

    if(type == POINT)
        geom.reset(new Point(link, root, link_data));
    else if(type == LINE)
        geom.reset(new Line(link, root, link_data));
    else if(type == PLANE)
        geom.reset(new Plane(link, root, link_data));
    else if(type == FRAME)
        geom.reset(new Frame(link, root, link_data));
    else if(type == CAPSULE)
        geom.reset(new Capsule(link, root, link_data));
    else if(type == JOINT_POSITION)
        geom.reset(new JointPosition(link, root, link_data));
    else if(type == JOINT_LIMITS)
        geom.reset(new JointLimits(link, root, link_data));
    else if(type == CONE)
        geom.reset(new Cone(link, root, link_data));
    else if(type == CYLINDER)
        geom.reset(new Cylinder(link, root, link_data));
    else
    {
        ROS_ERROR("Task geometry type %d is invalid.",type);
        ROS_BREAK();
    }
    return geom;
}
//------------------------------------------------------------------------
Point::Point() : TaskGeometry()
{
    type_ = POINT;
    p_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Point::Point(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = POINT;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Point::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 3);
    link_data_.reset(new Eigen::VectorXd(link_data));

    p_.reset(new Eigen::Vector3d(link_data)); //the internal representation of a point is just the Point::link_data_
}
//------------------------------------------------------------------------
void Point::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    //Express the point in the root frame
    root_data_.reset(new Eigen::VectorXd( (*trans_l_r_) * (*p_) ));
}
//------------------------------------------------------------------------
void Point::addMarker(visualization_msgs::MarkerArray& markers)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = link_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = markers.markers.size();
    marker.lifetime = ros::Duration(0.1);
    geometry_msgs::Point p;
    p.x=(*p_)(0);
    p.y=(*p_)(1);
    p.z=(*p_)(2);
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
//void Point::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Point::computeWitnessPoints(...) not implemented yet!");
//}
//------------------------------------------------------------------------
Line::Line() : TaskGeometry()
{
    type_ = LINE;
    p_.reset(new Eigen::Vector3d);
    v_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Line::Line(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = LINE;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Line::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 6);
    link_data_.reset(new Eigen::VectorXd(link_data));

    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction
    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
    v_.reset(new Eigen::Vector3d(link_data.tail<3>()));
    v_->normalize(); //normalize the direction vector just to make sure
}
//------------------------------------------------------------------------
void Line::addMarker(visualization_msgs::MarkerArray& markers)
{
    //transformation which points x in the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitX() ,(*v_));

    visualization_msgs::Marker m;
    m.header.frame_id = link_;
    m.header.stamp = ros::Time::now();
    m.lifetime = ros::Duration(0.1);
    m.type =  visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = markers.markers.size();
    m.pose.position.x = (*p_)(0);
    m.pose.position.y = (*p_)(1);
    m.pose.position.z = (*p_)(2);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = LINE_SCALE;
    m.scale.y = 0.05 * LINE_SCALE;
    m.scale.z = 0.05 * LINE_SCALE;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    markers.markers.push_back(m);
}
//------------------------------------------------------------------------
void Line::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    root_data_.reset(new Eigen::VectorXd(6));
    //Express the line's reference point in the root frame
    root_data_->head<3>() = (*trans_l_r_) * (*p_);
    root_data_->tail<3>() = trans_l_r_->linear() * (*v_);
}
//------------------------------------------------------------------------
//void Line::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Line::computeWitnessPoints(...) not implemented yet!");
//}
//------------------------------------------------------------------------
Plane::Plane() : TaskGeometry(), d_(0.0)
{
    type_ = PLANE;
    n_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Plane::Plane(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = PLANE;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Plane::addMarker(visualization_msgs::MarkerArray& markers)
{
    //transformation which points x in the plane normal direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitX() ,(*n_));

    visualization_msgs::Marker m;

    //normal
    m.header.frame_id = link_;
    m.header.stamp = ros::Time::now();
    m.type =  visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.1);
    m.id = markers.markers.size();
    m.pose.position.x = (*n_)(0)*d_;
    m.pose.position.y = (*n_)(1)*d_;
    m.pose.position.z = (*n_)(2)*d_;
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = LINE_SCALE;
    m.scale.y = 0.05 * LINE_SCALE;
    m.scale.z = 0.05 * LINE_SCALE;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    markers.markers.push_back(m);

    //plane
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,(*n_));
    m.type = visualization_msgs::Marker::CUBE;
    m.id = markers.markers.size();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = PLANE_SCALE;
    m.scale.y = PLANE_SCALE;
    m.scale.z = 0.0001;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.4;
    markers.markers.push_back(m);
}
//------------------------------------------------------------------------
void Plane::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 4);

    n_.reset(new Eigen::Vector3d(link_data.head<3>())); //the normal is given as the first 3 entries of link_data
    d_=link_data.tail<1>()(0);

    //normalize just to be sure;
    d_ = d_/n_->norm(); n_->normalize();

    link_data_.reset(new Eigen::VectorXd(4));
    link_data_->head<3>()=(*n_);
    link_data_->tail<1>()(0)=d_;
}
//------------------------------------------------------------------------
void Plane::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    //Express the plane in the root frame:nR = rot_l_r*nL; dR=rot_l_r*nL*trans_l_r*(nL*dl)
    root_data_.reset(new Eigen::VectorXd(4));
    root_data_->head<3>() = trans_l_r_->linear() * (*n_);
    root_data_->tail<1>() = root_data_->head<3>().transpose() * ((*trans_l_r_) * ((*n_) * d_));

    //    Eigen::Hyperplane<double,3> h((*n_),d_); //the plane in the link frame
    //    h.transform( *trans_l_r_); //transform the plane to the root frame
}
//------------------------------------------------------------------------
//void Plane::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Plane::computeWitnessPoints(...) not implemented yet!");
//}
//------------------------------------------------------------------------
Capsule::Capsule() : TaskGeometry(), r_(0.0)
{
    type_ = CAPSULE;
    p_.reset(new Eigen::Vector3d);
    t_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Capsule::Capsule(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = CAPSULE;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Capsule::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 7);
    link_data_.reset(new Eigen::VectorXd(link_data));

    p_.reset(new Eigen::Vector3d(link_data.head<3>())); //the first 3 entries of link_data correspond to the capsule's start point
    t_.reset(new Eigen::Vector3d(link_data.segment<3>(3))); //the next 3 entries of link_data correspond to the capsule's end point
    r_ = link_data.tail<1>()(0);
}
//------------------------------------------------------------------------
void Capsule::addMarker(visualization_msgs::MarkerArray& markers)
{
    visualization_msgs::Marker m;

    //spheres
    m.header.frame_id = link_;
    m.header.stamp = ros::Time::now();
    m.type =  visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.1);
    m.id = markers.markers.size();
    m.pose.position.x = (*p_)(0);
    m.pose.position.y = (*p_)(1);
    m.pose.position.z = (*p_)(2);
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = r_;
    m.scale.y = r_;
    m.scale.z = r_;
    m.color.r = 1.0;
    m.color.g = 0.85;
    m.color.b = 0.0;
    m.color.a = 1.0;
    markers.markers.push_back(m);

    m.id = markers.markers.size();
    m.pose.position.x = (*t_)(0);
    m.pose.position.y = (*t_)(1);
    m.pose.position.z = (*t_)(2);
    markers.markers.push_back(m);

    //cylinder
    //transformation which points z in the capsule's cylinder direction
    Eigen::Quaterniond q;
    Eigen::Vector3d v=(*t_)-(*p_);
    ROS_ASSERT(v.norm() > 0.0); //spheres should be handled extra for now ...
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,v);
    m.type =  visualization_msgs::Marker::CYLINDER;
    m.id = markers.markers.size();
    m.pose.position.x = (*p_)(0)+v(0)/2;
    m.pose.position.y = (*p_)(1)+v(1)/2;
    m.pose.position.z = (*p_)(2)+v(2)/2;
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.z = v.norm();
    markers.markers.push_back(m);
}
//------------------------------------------------------------------------
void Capsule::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    root_data_.reset(new Eigen::VectorXd(7));
    //Express the capsules geometry in the root frame
    root_data_->head<3>() = (*trans_l_r_) * (*p_);
    root_data_->segment<3>(3) = (*trans_l_r_) * (*t_);
    root_data_->tail<1>()(0) = r_;
}
//------------------------------------------------------------------------
//void Capsule::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Capsule::computeWitnessPoints(...) not implemented yet!");
//}
//------------------------------------------------------------------------
Frame::Frame() : TaskGeometry()
{
    type_ = FRAME;
    trans_f_l_.reset(new Eigen::Affine3d);
}
//------------------------------------------------------------------------
Frame::Frame(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = FRAME;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Frame::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Quaterniond q(trans_f_l_->linear());
    visualization_msgs::Marker e;

    //e_x
    e.header.frame_id = link_;
    e.header.stamp = ros::Time::now();
    e.type = visualization_msgs::Marker::ARROW;
    e.action = visualization_msgs::Marker::ADD;
    e.lifetime = ros::Duration(0.1);
    e.id = markers.markers.size();
    e.pose.position.x = trans_f_l_->translation()(0);
    e.pose.position.y = trans_f_l_->translation()(1);
    e.pose.position.z = trans_f_l_->translation()(2);
    e.pose.orientation.x = q.x();
    e.pose.orientation.y = q.y();
    e.pose.orientation.z = q.z();
    e.pose.orientation.w = q.w();
    e.scale.x = LINE_SCALE;
    e.scale.y = 0.05 * LINE_SCALE;
    e.scale.z = 0.05 * LINE_SCALE;
    e.color.r = 1.0;
    e.color.g = 0.0;
    e.color.b = 0.0;
    e.color.a = 1.0;
    markers.markers.push_back(e);

    //e_y
    Eigen::Quaterniond qy(0.5, 0.5, 0.5, 0.5); //rotates from x to y
    qy=q*qy;
    e.id = markers.markers.size();
    e.pose.orientation.x = qy.x();
    e.pose.orientation.y = qy.y();
    e.pose.orientation.z = qy.z();
    e.pose.orientation.w = qy.w();
    e.color.r = 0.0;
    e.color.g = 1.0;
    e.color.b = 0.0;
    e.color.a = 1.0;
    markers.markers.push_back(e);

    //e_z
    Eigen::Quaterniond qz(0.5, -0.5, -0.5, -0.5); //rotates from x to z
    qz=q*qz;
    e.id = markers.markers.size();
    e.pose.orientation.x = qz.x();
    e.pose.orientation.y = qz.y();
    e.pose.orientation.z = qz.z();
    e.pose.orientation.w = qz.w();
    e.color.r = 0.0;
    e.color.g = 0.0;
    e.color.b = 1.0;
    e.color.a = 1.0;
    markers.markers.push_back(e);
}
//------------------------------------------------------------------------
void Frame::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 6);
    link_data_.reset(new Eigen::VectorXd(link_data));

    Eigen::Vector3d transl = link_data.head<3>();
    Eigen::Vector3d rpy = link_data.tail<3>();
    Eigen::Matrix3d rot;
    //create a x-y-z rotation matrix
    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    trans_f_l_.reset(new Eigen::Affine3d);
    trans_f_l_->translation() = transl;
    trans_f_l_->linear() = rot;
}
//------------------------------------------------------------------------
void Frame::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));
    Eigen::Affine3d trans_f_r( (*trans_l_r_) * (*trans_f_l_) ); //frame expressed in root

    root_data_.reset(new Eigen::VectorXd(6));
    root_data_->head<3>() = trans_f_r.translation();
    root_data_->tail<3>() = trans_f_r.linear().eulerAngles(0, 1, 2);
}
//------------------------------------------------------------------------
//void Frame::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
//{
//    ROS_WARN("Frame::computeWitnessPoints(...) not implemented yet!");
//}
//------------------------------------------------------------------------
JointPosition::JointPosition() : q_pos_(0.0)
{
    type_ = JOINT_POSITION;
    trans_j_l_.reset(new Eigen::Affine3d);
    trans_j_r_0_.reset(new Eigen::Affine3d);
}
//------------------------------------------------------------------------
JointPosition::JointPosition(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = JOINT_POSITION;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void JointPosition::setLinkData(Eigen::VectorXd const& link_data)
{
    //Joint position is described by a frame expressed in the link with z pointing in the joint axis and zero angle pointing in x
    //link_data.tail<6>() is the initial transformation of the joint to the root
    //the position angle q = link_data(0)
    ROS_ASSERT(link_data.rows() == 13);
    link_data_.reset(new Eigen::VectorXd(link_data));
    q_pos_ = link_data(0);
    Eigen::Vector3d transl = link_data.segment(1,3);
    Eigen::Vector3d rpy = link_data.segment(4,3);

    Eigen::Matrix3d rot;
    //create a x-y-z rotation matrix
    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    trans_j_l_.reset(new Eigen::Affine3d);
    trans_j_l_->translation() = transl;
    trans_j_l_->linear() = rot;

    trans_j_r_0_.reset(new Eigen::Affine3d);
    transl = link_data.segment(7,3);
    rpy = link_data.tail<3>();
    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    trans_j_r_0_->translation() = transl;
    trans_j_r_0_->linear() = rot;
}
//------------------------------------------------------------------------
void JointPosition::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));
    Eigen::Affine3d trans_j_r( (*trans_l_r_) * (*trans_j_l_) ); //joint frame expressed in root

    root_data_.reset(new Eigen::VectorXd(13));
    (*root_data_)(0) = q_pos_;
    root_data_->segment(1,3) = trans_j_r.translation();
    root_data_->segment(4,3) = trans_j_r.linear().eulerAngles(0, 1, 2);
    root_data_->tail<6>() = link_data_->tail<6>(); //initial transform from joint to root doesn't change
}
//------------------------------------------------------------------------
void JointPosition::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Quaterniond q(trans_j_r_0_->linear());
    visualization_msgs::Marker a;

    //transformation which points x in z direction for plotting the arrow
    Eigen::Quaterniond q_rot;
    q_rot.setFromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    q_rot = q * q_rot;
    //joint axis
    a.header.frame_id = root_;
    a.header.stamp = ros::Time::now();
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.lifetime = ros::Duration(0.1);
    a.id = markers.markers.size();
    a.pose.position.x = trans_j_r_0_->translation()(0);
    a.pose.position.y = trans_j_r_0_->translation()(1);
    a.pose.position.z = trans_j_r_0_->translation()(2);
    a.pose.orientation.x = q_rot.x();
    a.pose.orientation.y = q_rot.y();
    a.pose.orientation.z = q_rot.z();
    a.pose.orientation.w = q_rot.w();
    a.scale.x = LINE_SCALE;
    a.scale.y = 0.05 * LINE_SCALE;
    a.scale.z = 0.05 * LINE_SCALE;
    a.color.r = 1.0;
    a.color.g = 0.4;
    a.color.b = 0.0;
    a.color.a = 1.0;
    markers.markers.push_back(a);

    //joint position indication line
    visualization_msgs::Marker l;
    geometry_msgs::Point p;
    l.header.frame_id = root_;
    l.header.stamp = ros::Time::now();
    l.type = visualization_msgs::Marker::LINE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    l.lifetime = ros::Duration(0.1);
    l.id = markers.markers.size();
    l.pose.position.x = trans_j_r_0_->translation()(0);
    l.pose.position.y = trans_j_r_0_->translation()(1);
    l.pose.position.z = trans_j_r_0_->translation()(2);
    l.pose.orientation.x = q.x();
    l.pose.orientation.y = q.y();
    l.pose.orientation.z = q.z();
    l.pose.orientation.w = q.w();
    l.scale.x = LINE_WIDTH;
    l.color.r = 1.0;
    l.color.g = 0.0;
    l.color.b = 1.0;
    l.color.a = 1.0;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    p.x = p.x + LINE_SCALE * cos(q_pos_);
    p.y = p.y + LINE_SCALE * sin(q_pos_);
    p.z = 0;
    l.points.push_back(p);
    markers.markers.push_back(l);

    //current joint position indication line
    q = Eigen::Quaterniond(trans_j_l_->linear());
    l = visualization_msgs::Marker();
    l.header.stamp = ros::Time::now();
    l.type = visualization_msgs::Marker::LINE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    l.header.frame_id = link_;
    l.id = markers.markers.size();
    l.pose.position.x = trans_j_l_->translation()(0);
    l.pose.position.y = trans_j_l_->translation()(1);
    l.pose.position.z = trans_j_l_->translation()(2);
    l.pose.orientation.x = q.x();
    l.pose.orientation.y = q.y();
    l.pose.orientation.z = q.z();
    l.pose.orientation.w = q.w();
    l.scale.x = 1.2 * LINE_WIDTH;
    l.color.r = 0.0;
    l.color.g = 1.0;
    l.color.b = 1.0;
    l.color.a = 1.0;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    p.x = 0.8 * LINE_SCALE;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    markers.markers.push_back(l);
}
//------------------------------------------------------------------------
JointLimits::JointLimits()
{
    type_ = JOINT_LIMITS;
    trans_j_l_.reset(new Eigen::Affine3d);
    trans_j_r_0_.reset(new Eigen::Affine3d);
    lb_.reset(new Eigen::Vector3d);
    ub_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
JointLimits::JointLimits(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = JOINT_LIMITS;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void JointLimits::setLinkData(Eigen::VectorXd const& link_data)
{
    //Joint limits is described by a frame expressed in the link with z pointing in the joint axis and zero angle pointing in x
    //link_data.segment(6,6) is the initial transformation of the joint to the root
    //link_data.tail<6>() are the upper and lower bounds
    ROS_ASSERT(link_data.rows() == 18);
    link_data_.reset(new Eigen::VectorXd(link_data));

    Eigen::Matrix3d rot;
    Eigen::Vector3d transl = link_data.head<3>();
    Eigen::Vector3d rpy = link_data.segment(3,3);

    //transform from the joint frame to the link frame
    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    trans_j_l_.reset(new Eigen::Affine3d);
    trans_j_l_->translation() = transl;
    trans_j_l_->linear() = rot;

    //transform from the joint frame at q==0 to the root frame
    trans_j_r_0_.reset(new Eigen::Affine3d);
    transl = link_data.segment(6,3);
    rpy = link_data.segment(9,3);
    rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    trans_j_r_0_->translation() = transl;
    trans_j_r_0_->linear() = rot;

    //lower bounds q_min, q_mins and q_mini
    lb_.reset(new Eigen::Vector3d(link_data.segment(12,3)));
    //upper bounds q_max, q_maxs and q_maxi
    ub_.reset(new Eigen::Vector3d(link_data.tail<3>()));

    //Make sure the given limits are consistent
    ROS_ASSERT((*lb_)(2) <= (*ub_)(2));
    for(unsigned int i=0;  i<2; i++)
    {
        ROS_ASSERT((*lb_)(i+1) >= (*lb_)(i));
        ROS_ASSERT((*ub_)(i+1) <= (*ub_)(i));
    }
}
//------------------------------------------------------------------------
void JointLimits::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));
    Eigen::Affine3d trans_j_r( (*trans_l_r_) * (*trans_j_l_) ); //joint frame expressed in root

    root_data_.reset(new Eigen::VectorXd(18));
    root_data_->head<3>() = trans_j_r.translation();
    root_data_->segment(3,3) = trans_j_r.linear().eulerAngles(0, 1, 2);
    root_data_->tail<12>() = link_data_->tail<12>(); //initial transform from joint to root and the limits don't change
}
//------------------------------------------------------------------------
void JointLimits::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Quaterniond q(trans_j_r_0_->linear());
    visualization_msgs::Marker a;
    //transformation which points x in z direction for plotting the arrow
    Eigen::Quaterniond q_rot;
    q_rot.setFromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    q_rot = q * q_rot;
    //joint axis
    a.header.frame_id = root_;
    a.header.stamp = ros::Time::now();
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.lifetime = ros::Duration(0.1);
    a.id = markers.markers.size();
    a.pose.position.x = trans_j_r_0_->translation()(0);
    a.pose.position.y = trans_j_r_0_->translation()(1);
    a.pose.position.z = trans_j_r_0_->translation()(2);
    a.pose.orientation.x = q_rot.x();
    a.pose.orientation.y = q_rot.y();
    a.pose.orientation.z = q_rot.z();
    a.pose.orientation.w = q_rot.w();
    a.scale.x = LINE_SCALE;
    a.scale.y = 0.05 * LINE_SCALE;
    a.scale.z = 0.05 * LINE_SCALE;
    a.color.r = 1.0;
    a.color.g = 0.4;
    a.color.b = 0.0;
    a.color.a = 1.0;
    markers.markers.push_back(a);

    //lb, ub indication lines
    visualization_msgs::Marker l;
    geometry_msgs::Point p;
    l.header.frame_id = root_;
    l.header.stamp = ros::Time::now();
    l.type = visualization_msgs::Marker::LINE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    l.lifetime = ros::Duration(0.1);
    l.id = markers.markers.size();
    l.pose.position.x = trans_j_r_0_->translation()(0);
    l.pose.position.y = trans_j_r_0_->translation()(1);
    l.pose.position.z = trans_j_r_0_->translation()(2);
    l.pose.orientation.x = q.x();
    l.pose.orientation.y = q.y();
    l.pose.orientation.z = q.z();
    l.pose.orientation.w = q.w();
    l.scale.x = LINE_WIDTH;
    l.color.r = 1.0;
    l.color.g = 0.0;
    l.color.b = 0.0;
    l.color.a = 0.5;
    p.x =LINE_SCALE * cos((*lb_)(0));
    p.y =LINE_SCALE * sin((*lb_)(0));
    p.z = 0;
    l.points.push_back(p);
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    l.points.push_back(p);
    p.x =LINE_SCALE * cos((*ub_)(0));
    p.y =LINE_SCALE * sin((*ub_)(0));
    p.z = 0;
    l.points.push_back(p);
    p.x =LINE_SCALE * cos((*lb_)(1));
    p.y =LINE_SCALE * sin((*lb_)(1));
    p.z = 0;
    l.points.push_back(p);
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    l.points.push_back(p);
    p.x =LINE_SCALE * cos((*ub_)(1));
    p.y =LINE_SCALE * sin((*ub_)(1));
    p.z = 0;
    l.points.push_back(p);
    p.x =LINE_SCALE * cos((*lb_)(2));
    p.y =LINE_SCALE * sin((*lb_)(2));
    p.z = 0;
    l.points.push_back(p);
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    l.points.push_back(p);
    p.x =LINE_SCALE * cos((*ub_)(2));
    p.y =LINE_SCALE * sin((*ub_)(2));
    p.z = 0;
    l.points.push_back(p);

    std_msgs::ColorRGBA color;
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
    for(unsigned int i = 0; i<4; i++)
        l.colors.push_back(color);

    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
    for(unsigned int i = 0; i<4; i++)
        l.colors.push_back(color);

    color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
    for(unsigned int i = 0; i<4; i++)
        l.colors.push_back(color);

    markers.markers.push_back(l);

    //current joint position indication line
    q = Eigen::Quaterniond(trans_j_l_->linear());
    l = visualization_msgs::Marker();
    l.header.stamp = ros::Time::now();
    l.type = visualization_msgs::Marker::LINE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    l.lifetime = ros::Duration(0.1);
    l.header.frame_id = link_;
    l.id = markers.markers.size();
    l.pose.position.x = trans_j_l_->translation()(0);
    l.pose.position.y = trans_j_l_->translation()(1);
    l.pose.position.z = trans_j_l_->translation()(2);
    l.pose.orientation.x = q.x();
    l.pose.orientation.y = q.y();
    l.pose.orientation.z = q.z();
    l.pose.orientation.w = q.w();
    l.scale.x = 1.2 * LINE_WIDTH;
    l.color.r = 0.0;
    l.color.g = 1.0;
    l.color.b = 1.0;
    l.color.a = 1.0;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    p.x = 0.8 * LINE_SCALE;
    p.y = 0;
    p.z = 0;
    l.points.push_back(p);
    markers.markers.push_back(l);
}
//------------------------------------------------------------------------
Cone::Cone() : TaskGeometry(), alpha_(0.0)
{
    type_ = CONE;
    p_.reset(new Eigen::Vector3d);
    v_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Cone::Cone(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = CONE;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Cone::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 7);
    link_data_.reset(new Eigen::VectorXd(link_data));

    //first 3 entries of link_data are the cone's reference point, the second 3 entries the direction, the last entry is the opening angle
    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
    v_.reset(new Eigen::Vector3d(link_data.segment(3,3)));
    v_->normalize(); //normalize the direction vector just to make sure
    alpha_ = link_data(6);

    ROS_ASSERT((alpha_ <= 3.1416) && (alpha_ >= -3.1416)); //make sure the angle is between +/- PI
}
//------------------------------------------------------------------------
void Cone::addMarker(visualization_msgs::MarkerArray& markers)
{
    visualization_msgs::Marker m;

    //cylinder for the cone opening
    //transformation which points z in the cone's direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,(*v_));
    m.header.stamp = ros::Time::now();
    m.header.frame_id = link_;
    m.type =  visualization_msgs::Marker::CYLINDER;
    m.lifetime = ros::Duration(0.1);
    //    m.action = visualization_msgs::Marker::ADD;
    m.id = markers.markers.size();
    m.pose.position.x = (*p_)(0) + (*v_)(0) * cos(alpha_) * CONE_SCALE;
    m.pose.position.y = (*p_)(1) + (*v_)(1) * cos(alpha_) * CONE_SCALE;
    m.pose.position.z = (*p_)(2) + (*v_)(2) * cos(alpha_) * CONE_SCALE;
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.z = LINE_WIDTH;
    m.scale.x = 2 * CONE_SCALE * sin(alpha_);
    m.scale.y = 2 * CONE_SCALE * sin(alpha_);
    markers.markers.push_back(m);

    //line for the cylinder axis
    m.type =  visualization_msgs::Marker::LINE_LIST;
    m.id = markers.markers.size();
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    geometry_msgs::Point p;
    p.x = (*p_)(0);
    p.y = (*p_)(1);
    p.z = (*p_)(2);
    m.points.push_back(p);
    p.x = (*p_)(0) + (*v_)(0) * cos(alpha_) * CONE_SCALE;
    p.y = (*p_)(1) + (*v_)(1) * cos(alpha_) * CONE_SCALE;
    p.z = (*p_)(2) + (*v_)(2) * cos(alpha_) * CONE_SCALE;
    m.points.push_back(p);
    m.scale.x =  LINE_WIDTH;
    markers.markers.push_back(m);
}
//------------------------------------------------------------------------
void Cone::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    root_data_.reset(new Eigen::VectorXd(7));
    //Express the Cone's reference point in the root frame
    root_data_->head<3>() = (*trans_l_r_) * (*p_);
    root_data_->segment(3,3) = trans_l_r_->linear() * (*v_);
    (*root_data_)(6) = alpha_; //opening angle doesn't change
}
//------------------------------------------------------------------------
Cylinder::Cylinder() : TaskGeometry(), r_(0.0)
{
    type_ = CYLINDER;
    p_.reset(new Eigen::Vector3d);
    v_.reset(new Eigen::Vector3d);
}
//------------------------------------------------------------------------
Cylinder::Cylinder(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data) : TaskGeometry(link, root)
{
    type_ = CYLINDER;
    setLinkData(link_data);
}
//------------------------------------------------------------------------
void Cylinder::setLinkData(Eigen::VectorXd const& link_data)
{
    ROS_ASSERT(link_data.rows() == 7);
    link_data_.reset(new Eigen::VectorXd(link_data));

    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction, the last one is the radius
    p_.reset(new Eigen::Vector3d(link_data.head<3>()));
    v_.reset(new Eigen::Vector3d(link_data.segment(3,3)));
    v_->normalize(); //normalize the direction vector just to make sure
    r_ = link_data.tail<1>()(0);
}
//------------------------------------------------------------------------
void Cylinder::addMarker(visualization_msgs::MarkerArray& markers)
{
    //transformation which points z in the cylinder direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ() ,(*v_));

    visualization_msgs::Marker m;
    m.header.frame_id = link_;
    m.header.stamp = ros::Time::now();
    m.type =  visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.1);
    m.id = markers.markers.size();
    m.pose.position.x = (*p_)(0) + (*v_)(0) * 0.5 * LINE_SCALE;
    m.pose.position.y = (*p_)(1) + (*v_)(1) * 0.5 * LINE_SCALE;
    m.pose.position.z = (*p_)(2) + (*v_)(2) * 0.5 * LINE_SCALE;
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = 2 * r_;
    m.scale.y = 2 * r_;
    m.scale.z = LINE_SCALE;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 0.8;
    markers.markers.push_back(m);
}
//------------------------------------------------------------------------
void Cylinder::setLinkTransform(Eigen::Affine3d const& trans_l_r)
{
    trans_l_r_.reset(new Eigen::Affine3d(trans_l_r));

    root_data_.reset(new Eigen::VectorXd(7));
    //Express the line's reference point in the root frame
    root_data_->head<3>() = (*trans_l_r_) * (*p_);
    root_data_->segment(3,3) = trans_l_r_->linear() * (*v_);
    (*root_data_)(6) = r_; //transform doesn't change the radius
}
//------------------------------------------------------------------------
} //end namespace hqp_controllers
