#include <hqp_controllers/task_geometry.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

namespace hqp_controllers {

//----------------------------------------------------
TaskGeometry::TaskGeometry() : frame_(""), type_(UNDEFINED_GEOMETRY) {}
//----------------------------------------------------
TaskGeometry::TaskGeometry(std::string frame) : frame_(frame), type_(UNDEFINED_GEOMETRY) {}
//----------------------------------------------------
TaskGeometry::TaskGeometry(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset) : frame_(frame), offset_(offset), type_(UNDEFINED_GEOMETRY) {}
//----------------------------------------------------
void TaskGeometry::setFrame(std::string frame) {frame_ = frame;}
//----------------------------------------------------
void TaskGeometry::setOffset(const boost::shared_ptr<Eigen::Affine3d> offset) {offset_ = offset;}
//----------------------------------------------------
std::string TaskGeometry::getFrame() const {return frame_;}
//----------------------------------------------------
TaskGeometryType TaskGeometry::getType() const {return type_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Affine3d> TaskGeometry::getOffset() const {return offset_;}
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
Point::Point(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset) : TaskGeometry(frame, offset)
{
    type_=POINT;
}
//----------------------------------------------------
Capsule::Capsule() : TaskGeometry(), radius_(0), length_(0)
{
    type_=CAPSULE;
}
//----------------------------------------------------
Capsule::Capsule(std::string frame) : TaskGeometry(frame), radius_(0), length_(0)
{
    type_=CAPSULE;
}
//----------------------------------------------------
Capsule::Capsule(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset) : TaskGeometry(frame, offset), radius_(0), length_(0)
{
    type_=CAPSULE;
}
//----------------------------------------------------
void Capsule::setRadius(double radius) {radius_ = radius;}
//----------------------------------------------------
void Capsule::setLength(double length) {length_ = length;}
//----------------------------------------------------
double Capsule::getRadius() const {return radius_;}
//----------------------------------------------------
double Capsule::getLength() const {return length_;}
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
Plane::Plane(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset) : TaskGeometry(frame, offset)
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
