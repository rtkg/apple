#ifndef TASK_GEOMETRY_H
#define TASK_GEOMETRY_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>


namespace hqp_controllers {
//----------------------------------------------------------------------------------------------------
enum TaskGeometryType {BASIC_GEOMETRY = 0, POINT = 1, LINE = 2, PLANE = 3, FRAME = 4, CAPSULE = 5};
#define POINT_SCALE 0.02
#define LINE_SCALE  0.15
#define PLANE_SCALE 1.0
//----------------------------------------------------------------------------------------------------
class TaskGeometry
{
public:

    TaskGeometry();
    TaskGeometry(std::string const& link, std::string const& root);

    void setLink(std::string const& link);
    void setRoot(std::string const& root);

    std::string getLink() const;
    std::string getRoot() const;
    TaskGeometryType getType() const;
    boost::shared_ptr<Eigen::VectorXd> getLinkData() const;
    boost::shared_ptr<Eigen::VectorXd> getRootData() const;
    boost::shared_ptr<Eigen::Affine3d> getLinkTransform() const;

    friend std::ostream& operator<<(std::ostream& str, TaskGeometry const& geom);

    static boost::shared_ptr<TaskGeometry>  makeTaskGeometry(TaskGeometryType type, std::string const& link, std::string const& root, Eigen::VectorXd const& link_data); ///<factory method
    //**Sets the TaskGeometry::link_data_ member and updates the internatl representation of the data in the subclasses */
    virtual void setLinkData(Eigen::VectorXd const& link_data) = 0;
    //**Sets the TaskGeometry::trans_l_r_ member and updates the TaskGeometry::root_data_ member in the subclasses */
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r) = 0;
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const = 0;
    virtual void addMarker(visualization_msgs::MarkerArray& markers) = 0;

protected:

    TaskGeometryType type_;
    std::string link_;
    std::string root_;
    boost::shared_ptr<Eigen::VectorXd> link_data_; ///< the geometry data expressed in the link frame
    boost::shared_ptr<Eigen::VectorXd> root_data_; ///< the geometry data expressed in the root frame
    boost::shared_ptr<Eigen::Affine3d> trans_l_r_; ///< the transformation fromt the link frame to the root frame (= the pose of the link frame expressed in the root frame)
};
//------------------------------------------------------------------------------------------
class Point: public TaskGeometry
{
public:

    Point();
    Point(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

    virtual void setLinkData(Eigen::VectorXd const& link_data);
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

protected:

    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the point in the TaskGeometry::link_ frame
};
//------------------------------------------------------------------------------------------
class Line: public TaskGeometry
{
public:

    Line();
    Line(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

    virtual void setLinkData(Eigen::VectorXd const& link_data);
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
        virtual void addMarker(visualization_msgs::MarkerArray& markers);

protected:

    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the line's start point in the TaskGeometry::link_ frame
    boost::shared_ptr<Eigen::Vector3d> v_; ///< coordinates of the line's unit direction vector expressed in TaskGeometry::link_ frame
};
//------------------------------------------------------------------------------------------
class Plane: public TaskGeometry
{
public:

    Plane();
    Plane(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

    virtual void setLinkData(Eigen::VectorXd const& link_data);
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

protected:

    boost::shared_ptr<Eigen::Vector3d> n_; ///< coordinates of the plane's unit normal expressed in the TaskGeometry::link_ frame
    double d_; ///< plane offset
};
//------------------------------------------------------------------------------------------
class Capsule: public TaskGeometry
{
public:

    Capsule();
    Capsule(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

    virtual void setLinkData(Eigen::VectorXd const& link_data);
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
    virtual void addMarker(visualization_msgs::MarkerArray& markers);

protected:

    boost::shared_ptr<Eigen::Vector3d> p_; ///< coordinates of the capsule's start point expressed in TaskGeometry::link_ frame
    boost::shared_ptr<Eigen::Vector3d> t_; ///< coordinates of the capsule's end point expressed in TaskGeometry::link_ frame
    double r_; ///< capsule radius
};
//------------------------------------------------------------------------------------------
class Frame: public TaskGeometry
{
public:

    Frame();
    Frame(std::string const& link, std::string const& root, Eigen::VectorXd const& link_data);

    virtual void setLinkData(Eigen::VectorXd const& link_data);
    virtual void setLinkTransform(Eigen::Affine3d const& trans_l_r);
    virtual void computeWitnessPoints(Eigen::Matrix3d& pts,TaskGeometry const& geom) const;
      virtual void addMarker(visualization_msgs::MarkerArray& markers);

protected:

    boost::shared_ptr<Eigen::Affine3d> trans_f_l_; ///< transformation from the frame to the TaskGeometry::link_ frame (= pose of Frame::trans_f_l_ in the link frame)
};
//------------------------------------------------------------------------------------------

} //end namespace hqp_controllers

#endif // TASK_GEOMETRY_H


