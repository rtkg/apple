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
    void setFrame(std::string frame);

    std::string getFrame() const;
    TaskGeometryType getType() const;

protected:

    TaskGeometryType type_;
    std::string frame_;

//    boost::shared_ptr<Eigen::Affine3d> pose_;   ///< pose of the Geometry in the root frame (the chain base frame)
//    boost::shared_ptr<Eigen::MatrixXd> jacobian_; ///< jacobian of the geometry
};
//----------------------------------------------------------------------------------------------------
class Point: public TaskGeometry
{
public:

    Point();
    Point(std::string frame);
    Point(std::string frame, boost::shared_ptr<Eigen::Vector3d> p);

    void setPosition(boost::shared_ptr<Eigen::Vector3d> p);
    boost::shared_ptr<Eigen::Vector3d> getPosition() const;

protected:

    boost::shared_ptr<Eigen::Vector3d> p_; ///< position of the point expressed in frame

};
//----------------------------------------------------------------------------------------------------
class Capsule: public TaskGeometry
{
public:

    Capsule();
    Capsule(std::string frame);
    Capsule(std::string frame, boost::shared_ptr<Eigen::Vector3d> p, boost::shared_ptr<Eigen::Vector3d> t, double r);

    void setRadius(double radius);
    void setStartPosition(boost::shared_ptr<Eigen::Vector3d> p);
        void setEndPosition(boost::shared_ptr<Eigen::Vector3d> t);

    double getRadius() const;
    boost::shared_ptr<Eigen::Vector3d> getStartPosition() const;
        boost::shared_ptr<Eigen::Vector3d> getEndPosition() const;

private:

    boost::shared_ptr<Eigen::Vector3d> p_;
        boost::shared_ptr<Eigen::Vector3d> t_;
    double r_;

};
//----------------------------------------------------------------------------------------------------
class Plane: public TaskGeometry
{
public:

    Plane();
    Plane(std::string frame);
    Plane(std::string frame, boost::shared_ptr<Eigen::Vector3d> n, double d);

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
