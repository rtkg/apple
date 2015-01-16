#ifndef TASK_GEOMETRY_H
#define TASK_GEOMETRY_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace hqp_controllers {
//----------------------------------------------------------------------------------------------------
enum TaskGeometryType {UNDEFINED_GEOMETRY=1, POINT, PLANE, CAPSULE};
//----------------------------------------------------------------------------------------------------
class TaskGeometry
{
public:
    TaskGeometry();
    TaskGeometry(std::string frame);
    TaskGeometry(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset);
    void setFrame(std::string frame);
    void setOffset(const boost::shared_ptr<Eigen::Affine3d> offset);

    std::string getFrame() const;
    boost::shared_ptr<Eigen::Affine3d> getOffset() const;
    TaskGeometryType getType() const;

protected:

    TaskGeometryType type_;
    std::string frame_; ///< parent frame
    boost::shared_ptr<Eigen::Affine3d> offset_; ///<  offset transform from the parent frame
//    boost::shared_ptr<Eigen::Affine3d> pose_;   ///< pose of the Geometry in the root frame (the chain base frame)
//    boost::shared_ptr<Eigen::MatrixXd> jacobian_; ///< jacobian of the geometry
};
//----------------------------------------------------------------------------------------------------
class Point: public TaskGeometry
{
public:

    Point();
    Point(std::string frame);
    Point(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset);
};
//----------------------------------------------------------------------------------------------------
class Capsule: public TaskGeometry
{
public:

    Capsule();
    Capsule(std::string frame);
    Capsule(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset);

    void setRadius(double radius);
    void setLength(double length);

    double getRadius() const;
    double getLength() const;

private:

    double radius_;
    double length_;
};
//----------------------------------------------------------------------------------------------------
class Plane: public TaskGeometry
{
public:

    Plane();
    Plane(std::string frame);
    Plane(std::string frame, boost::shared_ptr<Eigen::Affine3d> offset);

    void setNormal(boost::shared_ptr<Eigen::Vector3d> n);
    void setOffset(double b);

    boost::shared_ptr<Eigen::Vector3d> getNormal()const;
    double getOffset() const;

private:

  boost::shared_ptr<Eigen::Vector3d> n_; ///< convention: n^T*x - d = 0, n: outward-pointing unit normal
  double d_;

};
//----------------------------------------------------------------------------------------------------

#endif // TASK_GEOMETRY_H

} //end namespace hqp_controllers
