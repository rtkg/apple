#include <hqp_controllers/hqp_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

template <class T>
void forward_command_controller::ForwardCommandController<T>::starting(const ros::Time& time)
{
  // Start controller with 0.0 effort
  command_ = 0.0;
}


PLUGINLIB_EXPORT_CLASS(effort_controllers::JointEffortController,controller_interface::ControllerBase)
