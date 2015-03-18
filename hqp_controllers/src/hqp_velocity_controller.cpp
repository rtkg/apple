#include <hqp_controllers/hqp_velocity_controller.h>
#include <hqp_controllers/task_geometry.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Geometry>
#include <hqp_controllers/utilities.h>
#include <hqp_controllers/utilities.h>

namespace hqp_controllers
{
//-----------------------------------------------------------------------
HQPVelocityController::HQPVelocityController() : active_(false) {}
//-----------------------------------------------------------------------
HQPVelocityController::~HQPVelocityController()
{
    set_tasks_srv_.shutdown();
    //    vis_t_obj_srv_.shutdown();
}
//-----------------------------------------------------------------------
bool HQPVelocityController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
{
    hqp_controllers_msgs::Task t_description;
    Task::taskMessageToXmlRpcValue(t_description);

    n_ = n;
    // Get the list of controlled joints
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

    //Construct a kinematic tree from the robot description on the parameter server
    std::string searched_param;
    std::string robot_description;
    if(n.searchParam("robot_description",searched_param))
    {
        n.getParam(searched_param,robot_description);
        ROS_ASSERT(kdl_parser::treeFromString(robot_description, k_tree_));
        ROS_INFO("KDL tree constructed. The included elements are:");
        KDL::SegmentMap segs = k_tree_.getSegments();
        for ( KDL::SegmentMap::const_iterator it=segs.begin(); it!=segs.end(); ++it)
            ROS_INFO("%s",it->second.segment.getName().c_str());
    }
    else
    {
        ROS_ERROR("Failed to construct KDL tree.");
        return false;
    }


    //============================================== REGISTER CALLBACKS =========================================
    activate_hqp_control_srv_ = n.advertiseService("activate_hqp_control",&HQPVelocityController::activateHQPControl,this);
    reset_hqp_control_srv_ = n.advertiseService("reset_hqp_control",&HQPVelocityController::resetHQPControl,this);
    set_tasks_srv_ = n.advertiseService("set_tasks",&HQPVelocityController::setTasks,this);
    load_tasks_srv_= n.advertiseService("load_tasks",&HQPVelocityController::loadTasks,this);
    remove_tasks_srv_= n.advertiseService("remove_tasks",&HQPVelocityController::removeTasks,this);
    vis_t_geom_srv_ = n.advertiseService("visualize_task_geometries",&HQPVelocityController::visualizeTaskGeometries, this);
    //============================================== REGISTER CALLBACKS END =========================================
    vis_t_geom_pub_.init(n, "task_geometries", 1);
    t_status_pub_.init(n, "task_status_array", 1);

    return true;
}

//-----------------------------------------------------------------------
void HQPVelocityController::starting(const ros::Time& time)
{
    // Start controller with 0.0 velocities
    commands_.resize(n_joints_);
    commands_.setZero();
}

///////////////
// CALLBACKS //
///////////////

//------------------------------------------------------------------------
bool HQPVelocityController::setTasks(hqp_controllers_msgs::SetTasks::Request & req, hqp_controllers_msgs::SetTasks::Response &res)
{
    lock_.lock();
    if(active_)
    {
        ROS_ERROR("HQP control is active: cannot set tasks!");
        lock_.unlock();
        res.success = false;
        return false;
    }

    for(unsigned int i = 0; i<req.tasks.size(); i++)
    {
        unsigned int id = task_manager_.getValidTaskId();
        XmlRpc::XmlRpcValue t_description = Task::taskMessageToXmlRpcValue(req.tasks[i]);
        boost::shared_ptr<Task> task = Task::makeTask(id, t_description, k_tree_, joints_);
        if(!task_manager_.addTask(task))
        {
            ROS_ERROR("Error in HQPVelocityController::setTasks(...): could not add task!");
            lock_.unlock();
            res.success = false;
            return false;
        }
        res.ids.push_back(id);
    }

    lock_.unlock();
    ROS_INFO("Sucessfully set %d tasks.", (int)req.tasks.size());
    res.success = true;
    return true;
}
//------------------------------------------------------------------------
bool HQPVelocityController::loadTasks(hqp_controllers_msgs::LoadTasks::Request & req, hqp_controllers_msgs::LoadTasks::Response &res)
{
    lock_.lock();
    if(active_)
    {
        ROS_ERROR("HQP control is active: cannot load tasks!");
        lock_.unlock();
        res.success = false;
        return false;
    }

    //FIND ON PARAMETER SERVER
    XmlRpc::XmlRpcValue t_definitions;
    std::string searched_param;
    if (!n_.searchParam(req.task_definitions, searched_param))
    {
        ROS_ERROR("Could not find task definition parameters %s on the parameter server!", req.task_definitions.c_str());
        lock_.unlock();
        res.success = false;
        return false;
    }

    n_.getParam(searched_param, t_definitions);
    ROS_ASSERT(t_definitions.getType() == XmlRpc::XmlRpcValue::TypeArray);

    //GENERATE TASKS
    for(unsigned int i=0; i<t_definitions.size();i++)
    {
        unsigned int id = task_manager_.getValidTaskId();
        boost::shared_ptr<Task> task = Task::makeTask(id, t_definitions[i], k_tree_, joints_);
        if(!task_manager_.addTask(task))
        {
            ROS_ERROR("Error in HQPVelocityController::loadTasks(...): could not add task!");
            lock_.unlock();
            res.success = false;
            return false;
        }
        res.ids.push_back(id);
    }

    ROS_INFO("Sucessfully loaded %d tasks.",t_definitions.size());

    lock_.unlock();
    res.success = true;
    return true;
}
//------------------------------------------------------------------------
bool HQPVelocityController::removeTasks(hqp_controllers_msgs::RemoveTasks::Request & req, hqp_controllers_msgs::RemoveTasks::Response &res)
{
    lock_.lock();
    if(active_)
    {
        ROS_ERROR("HQP control is active: cannot remove tasks!");
        lock_.unlock();
        res.success = false;
        return false;
    }

    for(unsigned int i=0; i<req.ids.size(); i++)
    {
        if(!task_manager_.removeTask(req.ids[i]))
        {
            lock_.unlock();
            res.success = false;
            return false;
        }

        for(unsigned int j=0; j<vis_ids_.size(); j++)  //check if the task is possibly visualized
            if(vis_ids_(j) == req.ids[i])
                removeRow(vis_ids_,j);
    }

    lock_.unlock();
    res.success = true;
    return true;
}
//-----------------------------------------------------------------------
bool HQPVelocityController::visualizeTaskGeometries(hqp_controllers_msgs::VisualizeTaskGeometries::Request & req, hqp_controllers_msgs::VisualizeTaskGeometries::Response &res)
{
    lock_.lock();
    if(active_)
    {
        ROS_ERROR("HQP control is active: cannot visualize task geometries!");
        lock_.unlock();
        res.success = false;
        return false;
    }

    vis_ids_.resize(req.ids.size());
    for(unsigned int i=0; i<req.ids.size();i++)
        vis_ids_(i)=req.ids[i];

    //check wether tasks with the requested ids exist
    boost::shared_ptr<Task> dummy;
    for(unsigned int i=0; i<vis_ids_.size(); i++)
        if(!task_manager_.getTask(vis_ids_(i), dummy))
        {
            res.success = false;
            ROS_ERROR("Task with id %d does not exist, cannot visualize.", vis_ids_(i));
            vis_ids_.resize(0);
        }
        else
            res.success = true;

    lock_.unlock();
    return res.success;
}
//-----------------------------------------------------------------------
bool HQPVelocityController::resetHQPControl(std_srvs::Empty::Request & req, std_srvs::Empty::Response &res)
{
    lock_.lock();
    if(active_)
    {
        ROS_ERROR("HQP control is active: cannot reset!");
        lock_.unlock();
        return false;
    }

    vis_ids_.resize(0);
    task_manager_.reset();
    lock_.unlock();

    ROS_INFO("Reset HQP control.");
    return true;
}
//-----------------------------------------------------------------------
bool HQPVelocityController::activateHQPControl(hqp_controllers_msgs::ActivateHQPControl::Request & req, hqp_controllers_msgs::ActivateHQPControl::Response &res)
{
    lock_.lock();
    active_ = req.active;
    lock_.unlock();

    if(req.active == true)
      ROS_INFO("HQP control activated.");
    else
      ROS_INFO("HQP control deactivated.");

    return true;
}
  //-----------------------------------------------------------------------
  void HQPVelocityController::update(const ros::Time& time, const ros::Duration& period)
  {
    lock_.lock();
    if(active_)
      {
        //compute jacobians and poses of the task links, as well as the task functions and jacobians
        task_manager_.updateTasks();

        //compute the HQP controls
        task_manager_.computeHQP();

        //set the computed task velocities if the computation was succesful, otherwise set them to zero
        if(!task_manager_.getDQ(commands_))
	  commands_.setZero();

        //std::cerr<<"computed DQ: "<<commands_.transpose()<<std::endl;
        //ROS_BREAK();

        // ================= DEBUG PRINT ============================
        //    for (int i=0; i<n_joints_; i++)
        //    {
        //        std::cout<<joints_->at(i).getName()<<std::endl;
        //        std::cout<<joints_->at(i).getPosition()<<std::endl;
        //        std::cout<<joints_->at(i).getVelocity()<<std::endl;
        //        std::cout<<joints_->at(i).getEffort()<<std::endl;
        //        std::cout<<std::endl;
        //    }
        // ================= DEBUG PRINT END ============================

        //======================= PUBLISH =================
        // limit rate of publishing
        if (PUBLISH_RATE > 0.0 && last_publish_time_ + ros::Duration(1.0/PUBLISH_RATE) < time)
	  {
            // we're actually publishing, so increment time
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0/PUBLISH_RATE);

            // try to publish the task object geometries
            // populate the message
            vis_t_geom_pub_.msg_.markers.clear();
            task_manager_.getTaskGeometryMarkers(vis_t_geom_pub_.msg_,vis_ids_);

            if (vis_t_geom_pub_.trylock())
	      vis_t_geom_pub_.unlockAndPublish();

            task_manager_.getTaskStatusArray(t_status_pub_.msg_);
            if (t_status_pub_.trylock())
	      t_status_pub_.unlockAndPublish();
	  }
      }
    else
      commands_.setZero();

    for(unsigned int i=0; i<n_joints_; i++)
      joints_.at(i).setCommand(commands_(i));

    lock_.unlock();
  }
  //-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPVelocityController,controller_interface::ControllerBase)
