#include <hqp_controllers/task_geometry.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>

namespace hqp_controllers {

//----------------------------------------------------
TaskGeometry::TaskGeometry() : frame_(""), type_(UNDEFINED_GEOMETRY) {}
//----------------------------------------------------
TaskGeometry::TaskGeometry(std::string frame) : frame_(frame), type_(UNDEFINED_GEOMETRY) {}
//----------------------------------------------------
void TaskGeometry::setFrame(std::string frame) {frame_ = frame;}
//----------------------------------------------------
std::string TaskGeometry::getFrame() const {return frame_;}
//----------------------------------------------------
TaskGeometryType TaskGeometry::getType() const {return type_;}
//----------------------------------------------------
Point::Point() : TaskGeometry()
{
    type_=POINT;
}
//----------------------------------------------------
Point::Point(std::string frame) : TaskGeometry(frame)
{
    type_=POINT;
}
//----------------------------------------------------
Point::Point(std::string frame, boost::shared_ptr<Eigen::Vector3d> p) : TaskGeometry(frame), p_(p)
{
    type_=POINT;
}
//----------------------------------------------------
void Point::setPosition(boost::shared_ptr<Eigen::Vector3d> p) {p_ = p;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Vector3d> Point::getPosition() const {return p_;}
//----------------------------------------------------
Capsule::Capsule() : TaskGeometry(), r_(0)
{
    type_=CAPSULE;
}
//----------------------------------------------------
Capsule::Capsule(std::string frame) : TaskGeometry(frame), r_(0)
{
    type_=CAPSULE;
}
//----------------------------------------------------
Capsule::Capsule(std::string frame, boost::shared_ptr<Eigen::Vector3d> p, boost::shared_ptr<Eigen::Vector3d> t, double r) : TaskGeometry(frame), p_(p), t_(t), r_(r)
{
    ROS_ASSERT(r_ >= 0);
    type_=CAPSULE;
}
//----------------------------------------------------
void Capsule::setRadius(double r)
{
    r_ = r;
    ROS_ASSERT(r_ >= 0);
}
//----------------------------------------------------
void Capsule::setStartPosition(boost::shared_ptr<Eigen::Vector3d> p) {p_ = p;}
//----------------------------------------------------
void Capsule::setEndPosition(boost::shared_ptr<Eigen::Vector3d> t) {t_ = t;}
//----------------------------------------------------
double Capsule::getRadius() const {return r_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Vector3d> Capsule::getStartPosition() const {return p_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Vector3d> Capsule::getEndPosition() const {return t_;}
//----------------------------------------------------
Plane::Plane() : TaskGeometry(), d_(0.0)
{
    type_=PLANE;
}
//----------------------------------------------------
Plane::Plane(std::string frame) : TaskGeometry(frame), d_(0.0)
{
    type_=PLANE;
}
//----------------------------------------------------
Plane::Plane(std::string frame, boost::shared_ptr<Eigen::Vector3d> n, double d) : TaskGeometry(frame), n_(n), d_(d)
{
    type_=PLANE;
}
//----------------------------------------------------
void Plane::setNormal(boost::shared_ptr<Eigen::Vector3d> n) {n_ = n;}
//----------------------------------------------------
void Plane::setOffset(double d) {d_ = d;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Vector3d> Plane::getNormal()const {return n_;}
//----------------------------------------------------
double Plane::getOffset() const {return d_;}
//----------------------------------------------------
} //end namespace hqp_controllers
