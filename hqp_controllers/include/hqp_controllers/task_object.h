#ifndef TASK_OBJECT_H
#define TASK_OBJECT_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace hqp_controllers {

enum TaskObjectType {UNDEFINED=1, POINT, PLANE, CAPSULE};

class TaskObject
{
public:
    TaskObject();
    TaskObject(unsigned int id);
    TaskObject(unsigned int id, std::string frame);

    void setFrame(std::string frame);
    void setOffset(boost::shared_ptr<Eigen::Affine3d> offset);

    std::string getFrame();
    boost::shared_ptr<Eigen::Affine3d> getOffset();

protected:
    unsigned int id_;
    TaskObjectType type_;
    std::string frame_; ///< parent frame
    boost::shared_ptr<Eigen::Affine3d> offset_; ///< constant offset transrform from the parent frame


};

#endif // TASK_OBJECT_H

} //end namespace hqp_controllers
