#include <hqp_controllers/task.h>

namespace hqp_controllers{
//---------------------------------------------------------
Task::Task() : priority_(0), name_(""), frame_(""), id_(0), type_(UNDEFINED), initialized_(false) {}
//---------------------------------------------------------
Task::Task(std::string name, unsigned int id) : priority_(0), name_(name), frame_(""), id_(id), type_(UNDEFINED), initialized_(false) {}
//---------------------------------------------------------
Task::Task(std::string name, std::string frame, unsigned int id) : priority_(0), name_(name), frame_(frame), id_(id), type_(UNDEFINED), initialized_(false) {}
//---------------------------------------------------------
std::string Task::getName() {return name_;}
//---------------------------------------------------------
unsigned int Task::getId() {return id_;}
//---------------------------------------------------------
TaskType Task::getType() {return type_;}
//---------------------------------------------------------
void Task::setName(std::string name) {name_=name;}
//---------------------------------------------------------
void Task::setId(unsigned int id) {id_=id;}
//---------------------------------------------------------
void Task::init(){initialized_ = true;}
//---------------------------------------------------------
bool Task::isInitialized() {return initialized_;}
//---------------------------------------------------------
unsigned int Task::getPriority() {return priority_;}
//---------------------------------------------------------
void Task::setPriority(unsigned int priority) {priority_= priority;}
//---------------------------------------------------------
} //end namespace hqp_controllers
