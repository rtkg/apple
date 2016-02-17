#include <hqp_controllers/task_geometry.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>
#include <math.h>
#include <typeinfo>
#include <hqp_controllers/utilities.h>

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
std::ostream& operator<<(std::ostream& str, OrientationQuantities const& ori)
{
    str<<"Orientation QUANTITIES"<<std::endl;
    str<<"h:"<<ori.h_<<std::endl;
    str<<"c: "<<ori.d_<<std::endl;
    str<<std::endl;
}
//------------------------------------------------------
ProjectionQuantities::ProjectionQuantities(Eigen::Matrix3Xd const& P1, Eigen::Matrix3Xd const& P2, Eigen::Matrix3Xd const& N, Eigen::VectorXd const& d) : P1_(P1), P2_(P2), N_(N), d_(d){}
//------------------------------------------------------
OrientationQuantities::OrientationQuantities(Eigen::RowVector3d const& h, double d) : h_(h), d_(d) {}
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

    return str;
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
void TaskGeometry::setLinkData(const Eigen::VectorXd &link_data){link_data_ = link_data;}
//------------------------------------------------------
void TaskGeometry::setTaskData(const Eigen::VectorXd &task_data){task_data_ = task_data;}
//------------------------------------------------------
boost::shared_ptr<TaskGeometry> TaskGeometry::makeTaskGeometry(TaskGeometryType type, std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    boost::shared_ptr<TaskGeometry> geom;

    if(type == POINT)
        geom.reset(new Point(link_frame, task_frame, link_data));
    else if(type == LINE)
        geom.reset(new Line(link_frame, task_frame, link_data));
    else if(type == PLANE)
        geom.reset(new Plane(link_frame, task_frame, link_data));
    //    else if(type == FRAME)
    //        geom.reset(new Frame(link, root, link_data));
    //    else if(type == CAPSULE)
    //        geom.reset(new Capsule(link, root, link_data));
    else if(type == JOINT_POSITION)
        geom.reset(new JointPosition(link_frame, task_frame, link_data));
    else if(type == JOINT_LIMITS)
        geom.reset(new JointLimits(link_frame, task_frame, link_data));
      else if(type == CONE)
        geom.reset(new Cone(link_frame, task_frame, link_data));
    else if(type == CYLINDER)
        geom.reset(new Cylinder(link_frame, task_frame, link_data));
    else if(type == SPHERE)
        geom.reset(new Sphere(link_frame, task_frame, link_data));
    else
    {
        ROS_ERROR("Task geometry type %d is invalid.",type);
        ROS_BREAK();
    }

    return geom;
}
//------------------------------------------------------------------------
ProjectableGeometry::ProjectableGeometry()
{
    link_frame_ = "";
    task_frame_ = "";
}
//------------------------------------------------------------------------
ProjectableGeometry::ProjectableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    link_frame_ = link_frame;
    task_frame_ = task_frame;
    link_data_ = link_data;
}
//------------------------------------------------------------------------
OrientableGeometry::OrientableGeometry()
{
    link_frame_ = "";
    task_frame_ = "";
}
//------------------------------------------------------------------------
OrientableGeometry::OrientableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    link_frame_ = link_frame;
    task_frame_ = task_frame;
    link_data_ = link_data;
}
//------------------------------------------------------------------------
Point::Point() : ProjectableGeometry()
{
    link_data_.resize(3);
    link_data_.setZero();
    task_data_.resize(3);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Point::Point(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
    ROS_ASSERT(link_data_.rows() == 3); //point is described by x/y/z coordinates
    task_data_.resize(3);
    task_data_.setZero();
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
    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    Eigen::VectorXd n = plane.getTaskData().head<3>();
    double o = plane.getTaskData()(3);

    proj.N_.col(0) = n;
    proj.P2_ << task_data_;
    proj.d_(0) = o - n.transpose()*proj.P2_.col(0);
    proj.P1_.col(0) = task_data_ + proj.d_(0)*n; //Projection of the point onto the plane

    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoCylinder(const Cylinder &cylinder) const
{
    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    Eigen::Vector3d c = cylinder.getTaskData().head<3>();
    Eigen::Vector3d v = cylinder.getTaskData().segment<3>(3);

    Eigen::Vector3d n(task_data_ - (c + v * v.dot(task_data_ - c)));
    proj.d_(0) = cylinder.getTaskData()(6) - n.norm();
    proj.P1_.col(0) = task_data_ - n;
    n.normalize();
    proj.N_.col(0) = n;
    proj.P2_ << task_data_;

    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoSphere(Sphere const& sphere)const
{
    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    proj.P1_ << sphere.getTaskData().head<3>();
    proj.P2_ << task_data_;
    proj.N_ << proj.P2_ - proj.P1_;
    proj.d_(0) = -proj.N_.norm()+sphere.getTaskData()(3);
    proj.N_.normalize();

    // std::cout<<"P1: "<<proj.P1_.transpose()<<std::endl;
    // std::cout<<"P2: "<<proj.P2_.transpose()<<std::endl;
    // std::cout<<"N: "<<proj.N_.transpose()<<std::endl;
    // std::cout<<"d: "<<proj.d_<<std::endl<<std::endl;
    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Point::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Point::projectOntoLine(Line const& line)const
{
    ROS_ERROR("Error in Point::projectOntoLine(...): not implemented yet!");
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
    marker.lifetime = ros::Duration(0.1);
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
Line::Line() : ProjectableGeometry()
{
    link_data_.resize(6);
    link_data_.setZero();
    task_data_.resize(6);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Line::Line(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    link_frame_ = link_frame;
    task_frame_ = task_frame;
    link_data_ = link_data;

    //link_data_(0-2) = point on line, link_data_(3-5) = line direction vector
    ROS_ASSERT(link_data_.rows() == 6);

    //normalize the direction vector just to be sure ...
    link_data_.tail<3>()=link_data_.tail<3>()/link_data_.tail<3>().norm();
    task_data_.resize(6);
    task_data_.setZero();
}
//------------------------------------------------------------------------
void Line::transformTaskData(Eigen::Affine3d const& T_l_t)
{
    task_data_.head<3>() = T_l_t * link_data_.head<3>(); //transform the point
    task_data_.tail<3>()=T_l_t.linear() * link_data_.tail<3>(); //rotate the line direction vector
}
//------------------------------------------------------------------------
OrientationQuantities Line::orient(const OrientableGeometry &geom) const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.orientTowardsLine(*this);
}
//------------------------------------------------------------------------
OrientationQuantities Line::coplanar(OrientableGeometry const& geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.coplanarWithLine(*this);
}
//------------------------------------------------------------------------
OrientationQuantities Line::orientTowardsLine(Line const& line)const
{
    ROS_ERROR("Line::orientTowardsLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Line::orientTowardsPlane(Plane const& plane)const
{
    OrientationQuantities ori;


    Eigen::Vector3d v1 = plane.getTaskData().head<3>();
    Eigen::Vector3d v2 = task_data_.segment<3>(3);

    ori.d_=fabs(v1.transpose()*v2);

    if(v1.transpose()*v2 < 0.0)
      {
	ori.h_ = v1.transpose()*skewSymmetricMatrix(v2);
      }
    else
      ori.h_ = -v1.transpose()*skewSymmetricMatrix(v2);

    return ori;

}
//------------------------------------------------------------------------
OrientationQuantities Line::orientTowardsCone(Cone const& cone)const
{
    OrientationQuantities ori;

    Eigen::Vector3d v1 = cone.getTaskData().segment<3>(3);
    Eigen::Vector3d v2 = task_data_.tail<3>();

    ori.d_ = cos(cone.getTaskData()(6)) - v1.transpose()*v2;
    ori.h_ = v1.transpose() * skewSymmetricMatrix(v2);

    return ori;
}
//------------------------------------------------------------------------
OrientationQuantities Line::coplanarWithLine(Line const& line)const
{
    ROS_ERROR("Line::coplanarWithLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Line::coplanarWithCone(Cone const& cone)const
{
    ROS_ERROR("Line::coplanarWithCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Line::project(const ProjectableGeometry &geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.projectOntoLine(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Line::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoLine(Line const& line)const
{
    Eigen::Vector3d l1 = line.getTaskData().head<3>();
    Eigen::Vector3d l2 = task_data_.head<3>();
    Eigen::Vector3d v1 = line.getTaskData().tail<3>();
    Eigen::Vector3d v2 = task_data_.tail<3>();

    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    //test for skewness
    if(v1.cross(v2).norm() < 1e-4)
    {
        //lines are parallel
        proj.N_.col(0) = l2-(l1+(v1*v1.dot(l2-l1)));
        proj.d_(0) = -proj.N_.col(0).norm();
        proj.N_.col(0).normalize();
        proj.P1_.col(0) = l1; //can be any point
        proj.P2_.col(0) = l1 - proj.N_.col(0) * proj.d_(0);
    }
    else
    {
        //lines are not parallel
        proj.N_.col(0) = v2.cross(v1);
        proj.N_.col(0).normalize();

        proj.d_(0) = proj.N_.col(0).dot(l1-l2);
//        if(proj.d_(0) > 0.0)
//        {
//            proj.d_(0) = -proj.d_(0);
//            proj.N_(0) = -proj.N_(0);
//        }

        //find the jacobian point
        Eigen::MatrixXd C(3,2);
        C.col(0) = v1; C.col(1) = -v2;

        Eigen::Vector2d lmbd = C.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(l2-proj.d_(0)*proj.N_.col(0)-l1);

        proj.P2_.col(0) = l2+lmbd(1)*v2;
        proj.P1_.col(0) = proj.P2_.col(0) + proj.N_.col(0) * proj.d_.col(0);

        //make sure the normal points from 2 to 1
        proj.N_.col(0) = proj.P2_.col(0)-proj.P1_.col(0);
        proj.N_.col(0).normalize();

        //make sure d has the right sign
        if(proj.d_(0) > 0.0)
            proj.d_(0) = -proj.d_(0);

    }

    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoPoint(const Point &point) const
{
    ROS_ERROR("Line::projectOntoPoint(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoPlane(const Plane &plane) const
{
    ROS_ERROR("Line::projectOntoPlane(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoCylinder(const Cylinder &cylinder) const
{
    ROS_ERROR("Line::projectOntoCylinder(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Line::projectOntoSphere(const Sphere &sphere) const
{
    ROS_ERROR("Error in Line::projectOntoSphere(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
void Line::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Vector3d p = link_data_.head<3>();
    Eigen::Vector3d v = link_data_.tail<3>();
    //transformation which points x in the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitX(), v);

    visualization_msgs::Marker marker;
    marker.header.frame_id = link_frame_;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(0.1);
    marker.type =  visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = markers.markers.size();
    marker.pose.position.x = p(0);
    marker.pose.position.y = p(1);
    marker.pose.position.z = p(2);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = LINE_SCALE;
    marker.scale.y = 0.05 * LINE_SCALE;
    marker.scale.z = 0.05 * LINE_SCALE;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
Plane::Plane() : ProjectableGeometry()
{
    link_data_.resize(4);
    link_data_.setZero();
    task_data_.resize(4);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Plane::Plane(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
    link_frame_ = link_frame;
    task_frame_ = task_frame;

    ROS_ASSERT(link_data_.rows() == 4); //plane is described by unit normal n and offset d

    //normalize on the plane normal just to be sure ...
    link_data_ = link_data_/link_data_.head<3>().norm();
    task_data_.resize(4);
    task_data_.setZero();
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
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Plane::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoPlane(const Plane &plane)const
{
    ROS_ERROR("Error in Plane::projectOntoPlane(...): Cannot project plane onto plane!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoLine(Line const& line)const
{
    ROS_ERROR("Error in Plane::projectOntoLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoCylinder(const Cylinder &cylinder) const
{
    ROS_ERROR("Plane::projectOntoCylinder(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Plane::projectOntoSphere(const Sphere &sphere) const
{
    ROS_ERROR("Error in Plane::projectOntoSphere(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Plane::orient(const OrientableGeometry &geom) const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    OrientationQuantities ori =  geom.orientTowardsPlane(*this);
    return geom.orientTowardsPlane(*this);
}
//------------------------------------------------------------------------
OrientationQuantities Plane::coplanar(OrientableGeometry const& geom)const
{
    ROS_ERROR("Plane::coplanar(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Plane::orientTowardsLine(Line const& line)const
{
    ROS_ERROR("Plane::orientTowardsLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Plane::orientTowardsCone(Cone const& cone)const
{
    ROS_ERROR("Plane::orientTowardsCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Plane::orientTowardsPlane(Plane const& plane)const
{
    ROS_ERROR("Plane::orientTowardsPlane(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
  OrientationQuantities Plane::coplanarWithLine(Line const& line)const
{
    ROS_ERROR("Plane::coplanarWithLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
  OrientationQuantities Plane::coplanarWithCone(Cone const& cone)const
{
    ROS_ERROR("Plane::coplanarWithCone(...): not implemented yet!");
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
//------------------------------------------------------------------------
JointLimits::JointLimits() : TaskGeometry(){}
//------------------------------------------------------------------------
JointLimits::JointLimits(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : TaskGeometry(link_frame, task_frame, link_data)
{
    ROS_ASSERT(link_data_.rows() == 3); //link_data_(0) = dq_max, link_data_(1) = upper joint limit, link_data_(2) = lower joint limit
    ROS_ASSERT(link_data_(0) >= 0.0); //dq_max has to be positive (-dq_max will be used in negative direction)
    ROS_ASSERT(link_data_(1) >= link_data_(2)); //upper joint limit has to be larger then the lower one
}
//------------------------------------------------------------------------
void JointLimits::transformTaskData(const Eigen::Affine3d &T_l_t)
{
    task_data_ = link_data_; //transformation ain't changing the joint limits
}
//------------------------------------------------------------------------
void JointLimits::addMarker(visualization_msgs::MarkerArray& markers)
{
    ROS_ERROR("Error in JointLimits::addMarker(...): not implemented yet!;");
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
}
//------------------------------------------------------------------------
Cylinder::Cylinder() : ProjectableGeometry()
{
    link_data_.resize(7);
    link_data_.setZero();
    task_data_.resize(7);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Cylinder::Cylinder(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
    //link_data_(0-2) = point on axis, link_data_(3-5) = axis direction vector, link_data_(6) = cylinder radius
    ROS_ASSERT(link_data_.rows() == 7);

    //normalize the direction vector just to be sure ...
    link_data_.segment<3>(3)=link_data_.segment<3>(3)/link_data_.segment<3>(3).norm();
    task_data_.resize(7);
    task_data_.setZero();
}
//------------------------------------------------------------------------
void Cylinder::transformTaskData(Eigen::Affine3d const& T_l_t)
{
    task_data_.head<3>() = T_l_t * link_data_.head<3>(); //transform the point
    task_data_.segment<3>(3)=T_l_t.linear() * link_data_.segment<3>(3); //rotate the axis vector
    task_data_(6) = link_data_(6); // cylinder radius is not affected by the transform
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::project(const ProjectableGeometry &geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.projectOntoCylinder(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoPoint(const Point &point) const
{
    ProjectionQuantities proj = point.projectOntoCylinder(*this);
    //    proj.N_ = proj.N_; inversion is taken into account by the jacobian difference in Projection::updateTask()
    proj.d_ = -proj.d_;

    Eigen::VectorXd p1 = proj.P2_.col(0);
    proj.P2_.col(0) = proj.P1_.col(0);
    proj.P1_.col(0) = p1;

    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoPlane(const Plane &plane) const
{
    ROS_ERROR("Cylinder::projectOntoPlane(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoCylinder(const Cylinder &cylinder) const
{
    ROS_ERROR("Plane::projectOntoCylinder(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoLine(Line const& line)const
{
    ROS_ERROR("Error in Cylinder::projectOntoLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Cylinder::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cylinder::projectOntoSphere(const Sphere &sphere) const
{
    ROS_ERROR("Error in Cylinder::projectOntoSphere(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
void Cylinder::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::Quaterniond q;
    Eigen::Vector3d p = link_data_.head<3>(); //cylinder point
    Eigen::Vector3d v = link_data_.segment<3>(3); //cylinder axis
    double r = link_data_(6); //cylinder radius

    //transformation which points z in the cylinder direction
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

    visualization_msgs::Marker marker;
    marker.header.frame_id = link_frame_;
    marker.header.stamp = ros::Time::now();
    marker.type =  visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);
    marker.id = markers.markers.size();
    marker.pose.position.x = p(0) + v(0) * 0.5 * LINE_SCALE;
    marker.pose.position.y = p(1) + v(1) * 0.5 * LINE_SCALE;
    marker.pose.position.z = p(2) + v(2) * 0.5 * LINE_SCALE;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 2 * r;
    marker.scale.y = 2 * r;
    marker.scale.z = LINE_SCALE;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
Sphere::Sphere() : ProjectableGeometry()
{
    link_data_.resize(4);
    link_data_.setZero();
    task_data_.resize(4);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Sphere::Sphere(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data) : ProjectableGeometry(link_frame, task_frame, link_data)
{
    ROS_ASSERT(link_data_.rows() == 4); //sphere is described by x/y/z coordinates and radius
    task_data_.resize(4);
    task_data_.setZero();
}
//------------------------------------------------------------------------
void Sphere::transformTaskData(Eigen::Affine3d const& T_l_t)
{
    task_data_.head<3>() = T_l_t * link_data_.head<3>();
    task_data_(3) = link_data_(3); //sphere radius ain't affected by the transformation
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::project(const ProjectableGeometry &geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.projectOntoSphere(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoLine(Line const& line)const
{
    ROS_ERROR("Error in Sphere::projectOntoLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Sphere::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoPoint(const Point &point) const
{

    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    proj.P1_ << point.getTaskData();
    proj.P2_ << task_data_.head<3>();
    proj.N_ << -proj.P2_ + proj.P1_;
    proj.d_(0) = proj.N_.norm()-task_data_(3);
    proj.N_.normalize();

    // std::cout<<"P1: "<<proj.P1_.transpose()<<std::endl;
    // std::cout<<"P2: "<<proj.P2_.transpose()<<std::endl;
    // std::cout<<"N: "<<proj.N_.transpose()<<std::endl;
    // std::cout<<"d: "<<proj.d_<<std::endl<<std::endl;
    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoPlane(const Plane &plane) const
{
    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    Eigen::VectorXd n = plane.getTaskData().head<3>();
    double o = plane.getTaskData()(3);

    proj.N_.col(0) = n;
    proj.P2_ << task_data_.head<3>();
    proj.d_(0) = o - n.transpose()*proj.P2_.col(0);
    proj.P1_.col(0) = task_data_.head<3>() + proj.d_(0)*n; //Projection of the point onto the plane
    proj.d_(0) += task_data_(3); //offset to account for the sphere radius - the only difference to a point/plane projection

    return proj;
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoCylinder(const Cylinder &cylinder) const
{
    ROS_ERROR("Sphere::projectOntoCylinder(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Sphere::projectOntoSphere(const Sphere &sphere) const
{
    ProjectionQuantities proj;
    proj.P1_.resize(Eigen::NoChange, 1);
    proj.P2_.resize(Eigen::NoChange, 1);
    proj.N_.resize(Eigen::NoChange, 1);
    proj.d_.resize(1);

    proj.P1_ << sphere.getTaskData().head<3>();
    proj.P2_ << task_data_.head<3>();
    proj.N_ << -proj.P1_ + proj.P2_;
    
    proj.d_(0) = -proj.N_.norm()+sphere.getTaskData()(3)+task_data_(3);
    proj.N_.normalize();

    // std::cerr<<"P1_: "<<proj.P1_.transpose()<<std::endl;
    // std::cerr<<"P2_: "<<proj.P2_.transpose()<<std::endl;
    // std::cerr<<"N_: "<<proj.N_.transpose()<<std::endl;
    // std::cerr<<"d_: "<<proj.d_<<std::endl;

    return proj;

    //    ROS_ERROR("Error in Sphere::projectOntoSphere(...): not implemented yet!");
    //    ROS_BREAK();
}
//------------------------------------------------------------------------
void Sphere::addMarker(visualization_msgs::MarkerArray& markers)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = link_frame_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = markers.markers.size();
    marker.lifetime = ros::Duration(0.1);
    marker.pose.position.x = link_data_.head<3>()(0);
    marker.pose.position.y = link_data_.head<3>()(1);
    marker.pose.position.z = link_data_.head<3>()(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2 * link_data_(3);
    marker.scale.y = 2 * link_data_(3);
    marker.scale.z = 2 * link_data_(3);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
Cone::Cone() : ProjectableGeometry()
{
    link_data_.resize(7);
    link_data_.setZero();
    task_data_.resize(7);
    task_data_.setZero();
}
//------------------------------------------------------------------------
Cone::Cone(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data)
{
    link_frame_ = link_frame;
    task_frame_ = task_frame;
    link_data_ = link_data;

    //link_data_(0-2) = point on cone axis, link_data_(3-5) = cone direction vector, link_data_(6) is cone opening angle
    ROS_ASSERT(link_data_.rows() == 7);

    //normalize the direction vector just to be sure ...
    link_data_.segment<3>(3)=link_data_.segment<3>(3)/link_data_.segment<3>(3).norm();
    task_data_.resize(7);
    task_data_.setZero();
}
//------------------------------------------------------------------------
void Cone::transformTaskData(Eigen::Affine3d const& T_l_t)
{
    task_data_.head<3>() = T_l_t * link_data_.head<3>(); //transform the point
    task_data_.segment<3>(3)=T_l_t.linear() * link_data_.segment<3>(3); //rotate the cone axis vector
    task_data_(6) = link_data_(6); //cone opening angle ain't affected by the transformation
}
//------------------------------------------------------------------------
OrientationQuantities Cone::orient(const OrientableGeometry &geom) const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.orientTowardsCone(*this);
}
//------------------------------------------------------------------------
OrientationQuantities Cone::coplanar(OrientableGeometry const& geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.coplanarWithCone(*this);
}
//------------------------------------------------------------------------
OrientationQuantities Cone::orientTowardsLine(Line const& line)const
{
    ROS_ERROR("Cone::orientTowardsLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
  OrientationQuantities Cone::orientTowardsPlane(Plane const& plane)const
  {
    OrientationQuantities ori;

    Eigen::Vector3d v1 = plane.getTaskData().head<3>();
    Eigen::Vector3d v2 = task_data_.segment<3>(3);

    ori.d_=cos(acos(fabs(v1.transpose()*v2))+task_data_(6));

    if(v1.transpose()*v2 < 0.0)
      {
	ori.h_ = v1.transpose()*skewSymmetricMatrix(v2);
      }
    else
      ori.h_ = -v1.transpose()*skewSymmetricMatrix(v2);

    return ori;
  }
//------------------------------------------------------------------------
OrientationQuantities Cone::orientTowardsCone(Cone const& cone)const
{
    ROS_ERROR("Cone::orientTowardsCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
OrientationQuantities Cone::coplanarWithLine(const Line &line) const
{
    OrientationQuantities ori;

    Eigen::Vector3d v2 = task_data_.segment<3>(3);
    Eigen::Vector3d v1 = v2;

    if(v2.cross(line.getTaskData().tail<3>()).norm() > 1e-4 )
    {
        //lines are not  parallel - find the projection by converting the cone to a line
        Line l2(link_frame_, task_frame_, link_data_.head<6>());
        l2.setTaskData(task_data_.head<6>());
        ProjectionQuantities l_proj = l2.projectOntoLine(line);

        v1 = l_proj.P1_.col(0) - task_data_.head<3>();
        v1.normalize();
        //std::cerr<<"Cone::coplanarWithLine(): l_proj:"<<l_proj<<std::endl<<"v1: "<<v1.transpose()<<std::endl<<"v2: "<<v2.transpose()<<std::endl;
    }

    ori.d_ = cos(task_data_(6)) - v1.transpose()*v2;
    ori.h_ = v1.transpose() * skewSymmetricMatrix(v2);

    return ori;
}
//------------------------------------------------------------------------
OrientationQuantities Cone::coplanarWithCone(Cone const& cone)const
{
    ROS_ERROR("Cone::coplanarWithCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::project(const ProjectableGeometry &geom)const
{
    ROS_ASSERT(task_frame_ == geom.getTaskFrame());
    return geom.projectOntoCone(*this);
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoCone(Cone const& cone)const
{
    ROS_ERROR("Error in Cone::projectOntoCone(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoLine(Line const& line)const
{
    ROS_ERROR("Cone:projectOntoLine(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoPoint(const Point &point) const
{
    ROS_ERROR("Cone::projectOntoPoint(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoPlane(const Plane &plane) const
{
    ROS_ERROR("Cone::projectOntoPlane(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoCylinder(const Cylinder &cylinder) const
{
    ROS_ERROR("Cone::projectOntoCylinder(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
ProjectionQuantities Cone::projectOntoSphere(const Sphere &sphere) const
{
    ROS_ERROR("Error in Cone::projectOntoSphere(...): not implemented yet!");
    ROS_BREAK();
}
//------------------------------------------------------------------------
void Cone::addMarker(visualization_msgs::MarkerArray& markers)
{
    Eigen::VectorXd p = link_data_.head<3>();
    Eigen::VectorXd v = link_data_.segment<3>(3);
    double alpha = link_data_(6);

    visualization_msgs::Marker marker;

    //cylinder for the cone opening
    //transformation which points z in the cone's direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = link_frame_;
    marker.type =  visualization_msgs::Marker::CYLINDER;
    marker.lifetime = ros::Duration(0.1);
    //    m.action = visualization_msgs::Marker::ADD;
    marker.id = markers.markers.size();
    marker.pose.position.x = p(0) + v(0) * cos(alpha) * CONE_SCALE;
    marker.pose.position.y = p(1) + v(1) * cos(alpha) * CONE_SCALE;
    marker.pose.position.z = p(2) + v(2) * cos(alpha) * CONE_SCALE;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    marker.scale.z = LINE_WIDTH;
    marker.scale.x = 2 * CONE_SCALE * sin(alpha);
    marker.scale.y = 2 * CONE_SCALE * sin(alpha);
    markers.markers.push_back(marker);

    //line for the cylinder axis
    marker.type =  visualization_msgs::Marker::LINE_LIST;
    marker.id = markers.markers.size();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    geometry_msgs::Point l;
    l.x = p(0);
    l.y = p(1);
    l.z = p(2);
    marker.points.push_back(l);
    l.x = p(0) + v(0) * cos(alpha) * CONE_SCALE;
    l.y = p(1) + v(1) * cos(alpha) * CONE_SCALE;
    l.z = p(2) + v(2) * cos(alpha) * CONE_SCALE;
    marker.points.push_back(l);
    marker.scale.x =  LINE_WIDTH;
    markers.markers.push_back(marker);
}
//------------------------------------------------------------------------
} //end namespace hqp_controllers
