#include <hqp_controllers/hqp_effort_controller.h>
#include <hqp_controllers/task_geometry.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Geometry>
#include <hqp_controllers/utilities.h>
#include <hqp_controllers/utilities.h>

namespace hqp_controllers
{
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
  //-----------------------------------------------------------------------
  HQPEffortController::HQPEffortController() : active_(false) {}
  //-----------------------------------------------------------------------
  HQPEffortController::~HQPEffortController()
  {
    set_tasks_srv_.shutdown();
    //    vis_t_obj_srv_.shutdown();
  }
  //-----------------------------------------------------------------------
  bool HQPEffortController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
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

    //initialize the PID controllers
    std::string pid_ = ("pid_");
    pids_.resize(n_joints_);
    for (unsigned int i=0; i<n_joints_;i++)
      if (!pids_[i].init(ros::NodeHandle(n, pid_ + joints_[i].getName())))
	{
	  ROS_ERROR("Error initializing the PID for joint %d",i);
	  return false;
	}

    // Start realtime state publishers
    for (unsigned int i=0; i<n_joints_;i++)
      {
	boost::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > ptr( new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, joints_[i].getName()+"/state", 1));
	c_state_pub_.push_back(ptr);
      }


    //============================================== REGISTER CALLBACKS =========================================
    activate_hqp_control_srv_ = n.advertiseService("activate_hqp_control",&HQPEffortController::activateHQPControl,this);
    reset_hqp_control_srv_ = n.advertiseService("reset_hqp_control",&HQPEffortController::resetHQPControl,this);
    set_tasks_srv_ = n.advertiseService("set_tasks",&HQPEffortController::setTasks,this);
    load_tasks_srv_= n.advertiseService("load_tasks",&HQPEffortController::loadTasks,this);
    remove_tasks_srv_= n.advertiseService("remove_tasks",&HQPEffortController::removeTasks,this);
    vis_t_geom_srv_ = n.advertiseService("visualize_task_geometries",&HQPEffortController::visualizeTaskGeometries, this);
    //============================================== REGISTER CALLBACKS END =========================================
    vis_t_geom_pub_.init(n, "task_geometries", 1);
    t_status_pub_.init(n, "task_status_array", 1);

    loop_count_=0;

    return true;
  }

  //-----------------------------------------------------------------------
  void HQPEffortController::starting(const ros::Time& time)
  {
    // Start controller with 0.0 velocities
    commands_.resize(n_joints_);
    commands_.setZero();
    for (unsigned int i=0; i<n_joints_;i++)
      pids_[i].reset();
  }

  ///////////////
  // CALLBACKS //
  ///////////////

  //------------------------------------------------------------------------
  bool HQPEffortController::setTasks(hqp_controllers_msgs::SetTasks::Request & req, hqp_controllers_msgs::SetTasks::Response &res)
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
            ROS_ERROR("Error in HQPEffortController::setTasks(...): could not add task!");
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
  bool HQPEffortController::loadTasks(hqp_controllers_msgs::LoadTasks::Request & req, hqp_controllers_msgs::LoadTasks::Response &res)
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
            ROS_ERROR("Error in HQPEffortController::loadTasks(...): could not add task!");
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
  bool HQPEffortController::removeTasks(hqp_controllers_msgs::RemoveTasks::Request & req, hqp_controllers_msgs::RemoveTasks::Response &res)
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
  bool HQPEffortController::visualizeTaskGeometries(hqp_controllers_msgs::VisualizeTaskGeometries::Request & req, hqp_controllers_msgs::VisualizeTaskGeometries::Response &res)
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
  bool HQPEffortController::resetHQPControl(std_srvs::Empty::Request & req, std_srvs::Empty::Response &res)
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
  bool HQPEffortController::activateHQPControl(hqp_controllers_msgs::ActivateHQPControl::Request & req, hqp_controllers_msgs::ActivateHQPControl::Response &res)
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
  void HQPEffortController::update(const ros::Time& time, const ros::Duration& period)
  {

    lock_.lock();
    if(active_)
      {
	if(loop_count_ % 1 == 0)
	  {

        //compute jacobians and poses of the task links, as well as the task functions and jacobians and compute the HQP
        task_manager_.updateTasks();

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
        //        std::cout<<joints_->at(i).getEffort()<<std::endl;
        //        std::cout<<joints_->at(i).getEffort()<<std::endl;
        //        std::cout<<std::endl;
        //    }
        // ================= DEBUG PRINT END ============================

	loop_count_=0;
	  }
	loop_count_++;
      }
    else
      commands_.setZero();


    // Set the PID error and compute the PID command with nonuniform time
    // step size. The derivative error is computed from the change in the error
    // and the timestep dt.
    for(unsigned int i=0; i<n_joints_; i++)
      {
	double error = commands_(i) - joints_[i].getVelocity();
        // if (fabs(error) < 1e-3)
        //   error=0;


	double commanded_effort = pids_[i].computeCommand(error, period);
	// if (i==6)
	// commanded_effort+=sgn(error)*1.1;
	// if (fabs(commanded_effort) > 2.0)
	//   commanded_effort = 2*sgn(commanded_effort);

	joints_[i].setCommand(commanded_effort);
      }

    if (PUBLISH_RATE > 0.0 && last_publish_time_ + ros::Duration(1.0/PUBLISH_RATE) < time)
      {
	//increment time
	last_publish_time_ = last_publish_time_ + ros::Duration(1.0/PUBLISH_RATE);

	//publish HQP stuff if active
	if(active_)
	  {
	    vis_t_geom_pub_.msg_.markers.clear();
            task_manager_.getTaskGeometryMarkers(vis_t_geom_pub_.msg_,vis_ids_);

            if (vis_t_geom_pub_.trylock())
	      vis_t_geom_pub_.unlockAndPublish();

            task_manager_.getTaskStatusArray(t_status_pub_.msg_);
            if (t_status_pub_.trylock())
	      t_status_pub_.unlockAndPublish();
	  }

	//always publish the tracking controller stuff
	for (unsigned int i=0; i<n_joints_;i++)
	  {
	    if (c_state_pub_[i]->trylock())
	      {
		c_state_pub_[i]->msg_.header.stamp = time;
		c_state_pub_[i]->msg_.set_point = commands_(i);
		c_state_pub_[i]->msg_.process_value_dot = (c_state_pub_[i]->msg_.process_value - joints_[i].getVelocity())/period.toSec();
		c_state_pub_[i]->msg_.process_value = joints_[i].getVelocity();
		c_state_pub_[i]->msg_.error = commands_(i) -joints_[i].getVelocity();
		c_state_pub_[i]->msg_.time_step = period.toSec();
		c_state_pub_[i]->msg_.command = joints_[i].getCommand();

		double dummy;
		  pids_[i].getGains(c_state_pub_[i]->msg_.p,
				    c_state_pub_[i]->msg_.i,
				    c_state_pub_[i]->msg_.d,
				    c_state_pub_[i]->msg_.i_clamp,
				    dummy);

		c_state_pub_[i]->unlockAndPublish();
	      }
	  }
      }

    lock_.unlock();
  }
  //-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPEffortController,controller_interface::ControllerBase)
