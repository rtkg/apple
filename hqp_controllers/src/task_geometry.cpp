#include <hqp_controllers/task_geometry.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>

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
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (*p_)(0);
    marker.pose.position.y = (*p_)(1);
    marker.pose.position.z = (*p_)(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = POINT_SCALE;
    marker.scale.y = POINT_SCALE;
    marker.scale.z = POINT_SCALE;
    marker.color.r = POINT_RGBA[0];
    marker.color.g = POINT_RGBA[1];
    marker.color.b = POINT_RGBA[2];
    marker.color.a = POINT_RGBA[3];

    markers.markers.push_back(marker);

    std::cout<<"Point: added marker..."<<std::endl;
}
//------------------------------------------------------------------------
void Point::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
{
    ROS_WARN("Point::computeWitnessPoints(...) not implemented yet!");
}
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
    ROS_WARN("Line::addMarker(...) is not implemented yet!");
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
void Line::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
{
    ROS_WARN("Line::computeWitnessPoints(...) not implemented yet!");
}
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
    ROS_WARN("Plane::addMarker(...) is not implemented yet!");
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

    Eigen::Hyperplane<double,3> h((*n_),d_); //the plane in the link frame
    h.transform( *trans_l_r_); //transform the plane to the root frame

    //Express the plane in the root frame
    root_data_.reset(new Eigen::VectorXd(4));
    root_data_->head<3>() = h.normal();
    root_data_->head<1>()(0) = h.offset();
}
//------------------------------------------------------------------------
void Plane::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
{
    ROS_WARN("Plane::computeWitnessPoints(...) not implemented yet!");
}
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
    ROS_WARN("Capsule::addMarker(...) is not implemented yet!");
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
void Capsule::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
{
    ROS_WARN("Capsule::computeWitnessPoints(...) not implemented yet!");
}
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
    visualization_msgs::Marker e;

    e.header.frame_id = link_;
    e.header.stamp = ros::Time();
    e.type = visualization_msgs::Marker::ARROW;
    e.action = visualization_msgs::Marker::ADD;
    e.pose.position.x = trans_f_l_->translation()(0);
    e.pose.position.y = trans_f_l_->translation()(1);
    e.pose.position.z = trans_f_l_->translation()(2);
    Eigen::Quaterniond q(trans_f_l_->linear());
    e.pose.orientation.x = q.x();
    e.pose.orientation.y = q.y();
    e.pose.orientation.z = q.z();
    e.pose.orientation.w = q.w();
    e.scale.x = FRAME_SCALE;
    e.scale.y = 0.1 * FRAME_SCALE;
    e.scale.z = 0.1 * FRAME_SCALE;
    e.color.r = 1.0;
    e.color.g = 0.0;
    e.color.b = 0.0;
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
void Frame::computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const
{
    ROS_WARN("Frame::computeWitnessPoints(...) not implemented yet!");
}
//------------------------------------------------------------------------

} //end namespace hqp_controllers
