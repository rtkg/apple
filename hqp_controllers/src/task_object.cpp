#include <hqp_controllers/task_object.h>

namespace hqp_controllers {

//----------------------------------------------------
TaskObject::TaskObject() : frame_(""), type_(UNDEFINED) {}
//----------------------------------------------------
TaskObject::TaskObject(unsigned int id) : id_(id), type_(UNDEFINED) {}
//----------------------------------------------------
TaskObject::TaskObject(unsigned int id, std::string frame) : id_(id), frame_(frame), type_(UNDEFINED) {}
//----------------------------------------------------
void TaskObject::setFrame(std::string frame) {frame_ = frame;}
//----------------------------------------------------
void TaskObject::setOffset(boost::shared_ptr<Eigen::Affine3d> offset) {offset_ = offset;}
//----------------------------------------------------
std::string TaskObject::getFrame() {return frame_;}
//----------------------------------------------------
boost::shared_ptr<Eigen::Affine3d> TaskObject::getOffset() {return offset_;}
//----------------------------------------------------

} //end namespace hqp_controllers
