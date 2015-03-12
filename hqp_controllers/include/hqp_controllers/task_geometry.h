#ifndef TASK_GEOMETRY_H
#define TASK_GEOMETRY_H

//#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>


namespace hqp_controllers {
//----------------------------------------------------------------------------------------------------
enum TaskGeometryType {POINT = 1, LINE = 2, PLANE = 3, FRAME = 4, CAPSULE = 5, JOINT_POSITION = 6, JOINT_LIMITS = 7, CONE = 8, CYLINDER = 9, SPHERE = 10};
#define POINT_SCALE  0.02
#define LINE_SCALE   0.3
#define PLANE_SCALE  0.8
#define CONE_SCALE   0.3
#define LINE_WIDTH   0.005

class Point;
class Plane;

//----------------------------------------------------------------------------------------------------
struct ProjectionQuantities
{
    Eigen::Matrix3Xd P1_; ///< Matrix holding the first set of projection points
    Eigen::Matrix3Xd P2_; ///< Matrix holding the second set of projection points
    Eigen::Matrix3Xd N_; ///< Matrix holding the projection normals
    Eigen::VectorXd d_; ///< Matrix holding the projection offsets

    ProjectionQuantities(){};
    ProjectionQuantities(Eigen::Matrix3Xd const& P1, Eigen::Matrix3Xd const& P2, Eigen::Matrix3Xd const& N, Eigen::VectorXd const& d);

    friend std::ostream& operator<<(std::ostream& str, ProjectionQuantities const& proj);
};
//----------------------------------------------------------------------------------------------------
class TaskGeometry
{
public:

    TaskGeometry();
    TaskGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);


    std::string getLinkFrame() const;
    std::string getTaskFrame() const;
    Eigen::VectorXd getLinkData() const;
    Eigen::VectorXd getTaskData() const;
    //    boost::shared_ptr<Eigen::Affine3d> getLinkTransform() const;

    friend std::ostream& operator<<(std::ostream& str, TaskGeometry const& geom);

    static boost::shared_ptr<TaskGeometry>  makeTaskGeometry(TaskGeometryType type, std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data); ///<factory method
    //**transforms the TaskGeometry::link_data_ into the given task frame and sets TaskGeometry::task_data_ accordingly*/
    virtual void transformTaskData(Eigen::Affine3d const& T_l_t) = 0;
    virtual void addMarker(visualization_msgs::MarkerArray& markers) = 0;

protected:

    std::string link_frame_;
    std::string task_frame_;
    Eigen::VectorXd link_data_; ///< the geometry data expressed in the link frame (constant)
    Eigen::VectorXd task_data_; ///< the geometry data expressed in the task frame
};
//------------------------------------------------------------------------------------------
class ProjectableGeometry: public TaskGeometry
{
  public:

    ProjectableGeometry();
    ProjectableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const = 0;
    virtual ProjectionQuantities projectOntoPoint(Point const& point)const = 0;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const = 0;
};
//------------------------------------------------------------------------------------------
class Point: public ProjectableGeometry
{
public:

    Point();
    Point(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;

 protected:

     virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
     virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;

    // boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the point in the TaskGeometry::link_ frame
};
//------------------------------------------------------------------------------------------
class JointPosition: public TaskGeometry
{
public:

    JointPosition();
    JointPosition(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

 protected:

};
//------------------------------------------------------------------------------------------
//class Sphere: public TaskGeometry
//{
//public:

//    Sphere();
//    Sphere(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the sphere's center point in the TaskGeometry::link_ frame
//    double r_; ///<sphere radius
//};
////------------------------------------------------------------------------------------------
//class Line: public TaskGeometry
//{
//public:

//    Line();
//    Line(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the line's start point in the TaskGeometry::link_ frame
//    boost::shared_ptr<Eigen::Vector3d> v_; ///< coordinates of the line's unit direction vector expressed in TaskGeometry::link_ frame
//};
////------------------------------------------------------------------------------------------
//** Plane defined as Plane::n_^T*x - Plane::d_ = 0.0 */
class Plane: public ProjectableGeometry
{
public:
Plane();
Plane(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

 virtual void transformTaskData(Eigen::Affine3d const& T_l_t);

 virtual void addMarker(visualization_msgs::MarkerArray& markers);
virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;

protected:

 virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
 virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;

//    boost::shared_ptr<Eigen::Vector3d> n_; ///< coordinates of the plane's unit normal expressed in the TaskGeometry::link_ frame
//    double d_; ///< plane offset
};
////------------------------------------------------------------------------------------------
//class Capsule: public TaskGeometry
//{
//public:

//    Capsule();
//    Capsule(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //   virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the capsule's start point expressed in TaskGeometry::link_ frame
//    boost::shared_ptr<Eigen::Vector3d> t_; ///< coordinates of the capsule's end point expressed in TaskGeometry::link_ frame
//    double r_; ///< capsule radius
//};
////------------------------------------------------------------------------------------------
//class Frame: public TaskGeometry
//{
//public:

//    Frame();
//    Frame(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //  virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Affine3d> trans_f_l_; ///< transformation from the frame to the TaskGeometry::link_ frame (= pose of Frame::trans_f_l_ in the link frame)

//};
//------------------------------------------------------------------------------------------
//class Cone: public TaskGeometry
//{
//public:

//    Cone();
//    Cone(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the cone's start point in the TaskGeometry::link_ frame
//    boost::shared_ptr<Eigen::Vector3d> v_; ///< coordinates of the cones's unit direction vector expressed in TaskGeometry::link_ frame
//    double alpha_; ///< the cone's opening angle
//};
////------------------------------------------------------------------------------------------
//class JointLimits: public TaskGeometry
//{
//public:
//    JointLimits();
//    JointLimits(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //  virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:
//    boost::shared_ptr<Eigen::Affine3d> trans_j_l_; ///< transformation from the joint frame to the TaskGeometry::link_ frame (= pose of Frame::trans_j_l_ in the link frame)
//    boost::shared_ptr<Eigen::Affine3d> trans_j_r_0_; ///< initial transformation from the joint frame to the TaskGeometry::root_ frame
//    boost::shared_ptr<Eigen::Vector3d> lb_; ///< vector of lower bounds holding q_min, q_mins and q_mini
//    boost::shared_ptr<Eigen::Vector3d> ub_; ///< vector of upper bounds holding q_max, q_maxs and q_maxi
//};
////------------------------------------------------------------------------------------------
//class Cylinder: public TaskGeometry
//{
//public:

//    Cylinder();
//    Cylinder(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //   virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the cylinder's start point expressed in TaskGeometry::link_ frame
//    boost::shared_ptr<Eigen::Vector3d> v_; ///< coordinates of the cylinder's unit direction vector expressed in TaskGeometry::link_ frame
//    double r_; ///< cylinder radius
//};
//------------------------------------------------------------------------------------------
} //end namespace hqp_controllers

#endif // TASK_GEOMETRY_H


