#include <hqp_controllers/hqp_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hqp_controllers
{
//-----------------------------------------------------------------------
HQPVelocityController::HQPVelocityController()
{
    commands_.clear();
}
//-----------------------------------------------------------------------
HQPVelocityController::~HQPVelocityController()
{
    sub_command_.shutdown();
}
//-----------------------------------------------------------------------
bool HQPVelocityController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
{
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
        return false;
    }
    n_joints_ = joint_names_.size();

    for(unsigned int i=0; i<n_joints_; i++)
    {
        try
        {
            joints_.push_back(hw->getHandle(joint_names_[i]));


        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }
    }

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &HQPVelocityController::commandCB, this);
    return true;
}
//-----------------------------------------------------------------------
void HQPVelocityController::starting(const ros::Time& time)
{
    // Start controller with 0.0 velocities
    commands_.resize(n_joints_, 0.0);
}
//-----------------------------------------------------------------------
void HQPVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
    for(unsigned int i=0; i<n_joints_; i++)
    {  joints_[i].setCommand(commands_[i]);  }
}
//-----------------------------------------------------------------------
void HQPVelocityController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size()!=n_joints_)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    for(unsigned int i=0; i<n_joints_; i++)
    {  commands_[i] = msg->data[i];  }
}
//-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPVelocityController,controller_interface::ControllerBase)
