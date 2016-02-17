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
#define PLANE_SCALE  4.8
#define CONE_SCALE   0.3
#define LINE_WIDTH   0.005

//----------------------------------------------------------------------------------------------------
//FORWARD DECLERATIONS
class Point;
class Plane;
class Cylinder;
class Sphere;
class Line;
class Cone;
//----------------------------------------------------------------------------------------------------
struct ProjectionQuantities
{
    Eigen::Matrix3Xd P1_; ///< Matrix holding the first set of projection points
    Eigen::Matrix3Xd P2_; ///< Matrix holding the second set of projection points
    Eigen::Matrix3Xd N_; ///< Matrix holding the projection normals
    Eigen::VectorXd d_; ///< Vector holding the projection offsets

    ProjectionQuantities(){};
    ProjectionQuantities(Eigen::Matrix3Xd const& P1, Eigen::Matrix3Xd const& P2, Eigen::Matrix3Xd const& N, Eigen::VectorXd const& d);

    friend std::ostream& operator<<(std::ostream& str, ProjectionQuantities const& proj);
};
//----------------------------------------------------------------------------------------------------
struct OrientationQuantities
{
    Eigen::RowVector3d h_; ///< direction vector
    double d_; ///< offset cosine

    OrientationQuantities(){};
    OrientationQuantities(Eigen::RowVector3d const& h, double d);

    friend std::ostream& operator<<(std::ostream& str, OrientationQuantities const& ori);
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

    void setLinkData(const Eigen::VectorXd& link_data);
    void setTaskData(const Eigen::VectorXd& task_data);
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
class ProjectableGeometry: public virtual TaskGeometry
{
public:

    ProjectableGeometry();
    ProjectableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const = 0;
    virtual ProjectionQuantities projectOntoPoint(Point const& point)const = 0;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const = 0;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const = 0;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const=0;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const=0;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const=0;
};
//------------------------------------------------------------------------------------------
class OrientableGeometry: public virtual TaskGeometry
{
public:

    OrientableGeometry();
    OrientableGeometry(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual OrientationQuantities orient(OrientableGeometry const& geom)const = 0;
    virtual OrientationQuantities orientTowardsLine(Line const& line)const = 0;
    virtual OrientationQuantities orientTowardsCone(Cone const& cone)const = 0;
virtual OrientationQuantities orientTowardsPlane(Plane const& cone)const = 0;

     virtual OrientationQuantities coplanar(OrientableGeometry const& geom)const = 0;
    virtual OrientationQuantities coplanarWithLine(Line const& line)const = 0;
         virtual OrientationQuantities coplanarWithCone(Cone const& cone)const = 0;
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

    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;
};
//------------------------------------------------------------------------------------------
class Sphere: public ProjectableGeometry
{
public:

    Sphere();
    Sphere(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;

    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;
};
//------------------------------------------------------------------------------------------
class Line: public ProjectableGeometry, OrientableGeometry
{
public:

    Line();
    Line(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;
    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;

    virtual OrientationQuantities orient(OrientableGeometry const& geom)const;
    virtual OrientationQuantities orientTowardsLine(Line const& line)const;
    virtual OrientationQuantities orientTowardsCone(Cone const& cone)const;
    virtual OrientationQuantities orientTowardsPlane(Plane const& cone)const;

    virtual OrientationQuantities coplanar(OrientableGeometry const& geom)const;
    virtual OrientationQuantities coplanarWithLine(Line const& line)const;
         virtual OrientationQuantities coplanarWithCone(Cone const& cone)const;
};
//------------------------------------------------------------------------------------------
//** Plane defined as Plane::n_^T*x - Plane::d_ = 0.0 */
 class Plane: public ProjectableGeometry, OrientableGeometry
{
public:
    Plane();
    Plane(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);

    virtual void addMarker(visualization_msgs::MarkerArray& markers);
    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;

    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;

  virtual OrientationQuantities orient(OrientableGeometry const& geom)const;
    virtual OrientationQuantities orientTowardsLine(Line const& line)const;
    virtual OrientationQuantities orientTowardsCone(Cone const& cone)const;
    virtual OrientationQuantities orientTowardsPlane(Plane const& cone)const;

    virtual OrientationQuantities coplanar(OrientableGeometry const& geom)const;
    virtual OrientationQuantities coplanarWithLine(Line const& line)const;
        virtual OrientationQuantities coplanarWithCone(Cone const& cone)const;
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
class Cone: public ProjectableGeometry, OrientableGeometry
{
public:
    Cone();
    Cone(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;
    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;

    virtual OrientationQuantities orient(OrientableGeometry const& geom)const;
    virtual OrientationQuantities orientTowardsLine(Line const& line)const;
    virtual OrientationQuantities orientTowardsCone(Cone const& cone)const;
    virtual OrientationQuantities orientTowardsPlane(Plane const& cone)const;

    virtual OrientationQuantities coplanar(OrientableGeometry const& geom)const;
    virtual OrientationQuantities coplanarWithLine(Line const& line)const;
        virtual OrientationQuantities coplanarWithCone(Cone const& cone)const;
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
};
//------------------------------------------------------------------------------------------
class JointLimits: public TaskGeometry
{
public:

    JointLimits();
    JointLimits(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);
    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

};
//------------------------------------------------------------------------------------------
class Cylinder: public ProjectableGeometry
{
public:
    Cylinder();
    Cylinder(std::string const& link_frame, std::string const& task_frame, Eigen::VectorXd const& link_data);

    virtual void transformTaskData(Eigen::Affine3d const& T_l_t);

    virtual void addMarker(visualization_msgs::MarkerArray& markers);
    virtual ProjectionQuantities project(ProjectableGeometry const& geom)const;
    virtual ProjectionQuantities projectOntoCone(Cone const& line)const;
    virtual ProjectionQuantities projectOntoPoint(Point const& point)const;
    virtual ProjectionQuantities projectOntoPlane(Plane const& plane)const;
    virtual ProjectionQuantities projectOntoCylinder(Cylinder const& cylinder)const;
    virtual ProjectionQuantities projectOntoSphere(Sphere const& sphere)const;
    virtual ProjectionQuantities projectOntoLine(Line const& line)const;
};
//------------------------------------------------------------------------------------------
} //end namespace hqp_controllers

#endif // TASK_GEOMETRY_H


