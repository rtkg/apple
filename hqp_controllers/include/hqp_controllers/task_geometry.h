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
//----------------------------------------------------------------------------------------------------
class TaskGeometry
{
public:

    TaskGeometry();
    TaskGeometry(std::string const& link);

//    void setLink(std::string const& link);

//    std::string getLink() const;
//    std::string getRoot() const;
//    TaskGeometryType getType() const;
//    boost::shared_ptr<Eigen::VectorXd> getLinkData() const;
//    boost::shared_ptr<Eigen::VectorXd> getRootData() const;
//    boost::shared_ptr<Eigen::Affine3d> getLinkTransform() const;

//    friend std::ostream& operator<<(std::ostream& str, TaskGeometry const& geom);

//    static boost::shared_ptr<TaskGeometry>  makeTaskGeometry(TaskGeometryType type, std::string const& link, std::string const& root, Eigen::VectorXd const& link_data); ///<factory method
//    //**Sets the TaskGeometry::link_data_ member and updates the internatl representation of the data in the subclasses */
//    virtual void setLinkData(Eigen::VectorXd const& link_data) = 0;
//    //**Sets the TaskGeometry::trans_l_r_ member and updates the TaskGeometry::root_data_ member in the subclasses */
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r) = 0;
//    //  virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const = 0;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers) = 0;

protected:

    std::string link_;
//    boost::shared_ptr<Eigen::VectorXd> link_data_; ///< the geometry data expressed in the link frame
//    boost::shared_ptr<Eigen::VectorXd> root_data_; ///< the geometry data expressed in the root frame (translated and rotated!)
//    boost::shared_ptr<Eigen::Affine3d> trans_l_r_; ///< the transformation fromt the link frame to the root frame (= the pose of the link frame expressed in the root frame)
};
//------------------------------------------------------------------------------------------
//class Point: public TaskGeometry
//{
//public:

//    Point();
//    Point(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    // virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the point in the TaskGeometry::link_ frame
//};
////------------------------------------------------------------------------------------------
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
////** Plane defined as Plane::n_^T*x - Plane::d_ = 0.0 */
//class Plane: public TaskGeometry
//{
//public:

//    Plane();
//    Plane(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:

//    boost::shared_ptr<Eigen::Vector3d> n_; ///< coordinates of the plane's unit normal expressed in the TaskGeometry::link_ frame
//    double d_; ///< plane offset
//};
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
////------------------------------------------------------------------------------------------
//class JointPosition: public TaskGeometry
//{
//public:
//    JointPosition();
//    JointPosition(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

//    virtual void setLinkData(Eigen::VectorXd const& link_data);
//    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
//    //  virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
//    virtual void addMarker(visualization_msgs::MarkerArray& markers);

//protected:
//    boost::shared_ptr<Eigen::Affine3d> trans_j_l_; ///< transformation from the joint frame to the TaskGeometry::link_ frame (= pose of Frame::trans_j_l_ in the link frame)
//    boost::shared_ptr<Eigen::Affine3d> trans_j_r_0_; ///< initial transformation from the joint frame to the TaskGeometry::root_ frame
//    double q_pos_; ///<Joint position value
//};
////------------------------------------------------------------------------------------------
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


