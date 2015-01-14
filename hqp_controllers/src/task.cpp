#include <hqp_controllers/task.h>

namespace hqp_controllers{
//---------------------------------------------------------
Task::Task() : name_(""), id_(0), type_(UNDEFINED) {}
//---------------------------------------------------------
Task::Task(std::string name, unsigned int id) : name_(name), id_(id), type_(UNDEFINED) {}
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

} //end namespace hqp_controllers
