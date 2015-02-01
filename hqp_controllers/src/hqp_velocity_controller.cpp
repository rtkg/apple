#include <hqp_controllers/hqp_velocity_controller.h>
#include <hqp_controllers/task_geometry.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Geometry>
#include <hqp_controllers/utilities.h>

namespace hqp_controllers
{
//-----------------------------------------------------------------------
HQPVelocityController::HQPVelocityController() : publish_rate_(TASK_OBJ_PUBLISH_RATE), active_(true)
{
    joints_.reset(new std::vector< hardware_interface::JointHandle >);
}
//-----------------------------------------------------------------------
HQPVelocityController::~HQPVelocityController()
{
    set_task_srv_.shutdown();
        set_task_obj_srv_.shutdown();
            vis_t_obj_srv_.shutdown();
}
//-----------------------------------------------------------------------
bool HQPVelocityController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
{
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
            joints_->push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }
    }

    //Construct a kinematic tree from the robot description on the parameter server
    boost::shared_ptr<KDL::Tree> k_tree(new KDL::Tree);
    std::string searched_param;
    std::string robot_description;
    if(n.searchParam("robot_description",searched_param))
    {
        n.getParam(searched_param,robot_description);
        ROS_ASSERT(kdl_parser::treeFromString(robot_description, *k_tree));
        ROS_INFO("KDL tree constructed. The included elements are:");
        KDL::SegmentMap segs = k_tree->getSegments();
        for ( KDL::SegmentMap::const_iterator it=segs.begin(); it!=segs.end(); ++it)
            ROS_INFO("%s",it->second.segment.getName().c_str());
    }
    else
    {
        ROS_ERROR("Failed to construct KDL tree.");
        return false;
    }

    task_manager_.setKinematicTree(k_tree);

    //Load the collision root from the parameter server
    std::string collision_root;
    if (n.searchParam("collision_root", searched_param))
        n.getParam(searched_param,collision_root);
    else
    {
        ROS_WARN("No collision root frame specified.");
    }

    //Load the collision objects from the parameter server
    XmlRpc::XmlRpcValue collision_objects;
    if (n.searchParam("collision_objects", searched_param))
    {
        n.getParam(searched_param,collision_objects);
        for (int32_t i = 0; i < collision_objects.size(); ++i)
        {
            std::string link = (std::string)collision_objects[i]["link"];
            boost::shared_ptr<KDL::Chain> chain(new KDL::Chain);
            if(!k_tree->getChain(collision_root,link,(*chain)))
            {
                ROS_ERROR("Could not get kinematic chain from %s to %s.", collision_root.c_str(),link.c_str());
                return false;
            }

            boost::shared_ptr<TaskObject> t_obj(new TaskObject(i,chain,collision_root,joints_)); //create a new task object
            unsigned int n_geom = collision_objects[i]["geometries"].size();

            ROS_ASSERT(n_geom >= 1); //make sure there is at least one task geometry associated with the newly created task object

            for(unsigned int j=0; j<n_geom; j++)
            {
                unsigned int n_data = collision_objects[i]["geometries"][j]["data"].size();
                Eigen::VectorXd link_data(n_data);
                for (unsigned int k = 0; k < n_data; k++)
                    link_data(k) = collision_objects[i]["geometries"][j]["data"][k];

                TaskGeometryType type = static_cast<TaskGeometryType>((int)collision_objects[i]["geometries"][j]["type"]);
                boost::shared_ptr<TaskGeometry> geom = TaskGeometry::makeTaskGeometry(type, link, collision_root, link_data);

                t_obj->addGeometry(geom);
            }
            t_obj->computeKinematics();
            task_manager_.addTaskObject(t_obj); //store the new object in the task manager
        }
        ROS_INFO("Collision objects loaded");
    }
    else
    {
        ROS_WARN("No collsion objects specified!");
    }

    //============================================== REGISTER CALLBACKS =========================================
    set_task_srv_ = n.advertiseService("set_tasks",&HQPVelocityController::setTasks,this);
    set_task_obj_srv_ = n.advertiseService("set_task_objects",&HQPVelocityController::setTaskObjects,this);
    vis_t_obj_srv_ = n.advertiseService("visualize_task_objects",&HQPVelocityController::visualizeTaskObjects,this);
    //============================================== REGISTER CALLBACKS END =========================================
    vis_t_obj_pub_.init(n, "task_objects", 1);
    t_statuses_pub_.init(n, "task_statuses", 1);

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

    for(unsigned int i = 0; i<req.tasks.size(); i++)
    {
        hqp_controllers_msgs::Task task = req.tasks[i];
        unsigned int n_t_obj = task.t_obj_ids.size();
        //make sure all task objects associated with the given task exist
        boost::shared_ptr<std::vector<TaskObject> > t_objs(new std::vector<TaskObject>(n_t_obj));
        for(unsigned int j=0; j<n_t_obj; j++)
        {
            TaskObject t_obj;
            if(!task_manager_.getTaskObject(task.t_obj_ids[j], t_obj))
            {
                res.success = false;
                lock_.unlock();
                ROS_ERROR("Cannot add task since the required task object with id %d does not exist in the task object map.", task.t_obj_ids[j]);
                return res.success;
            }
            t_objs->at(j) = t_obj;
        }

        //Read the data for the task dynamics
        Eigen::VectorXd data(task.dynamics.data.size());
        for(unsigned int j=0; j<data.rows();j++)
            data(j) = task.dynamics.data[j];

        TaskType task_type = static_cast<TaskType>(task.type);
        unsigned int id = task_manager_.getValidTaskId();
        TaskDynamicsType dynamics_type = static_cast<TaskDynamicsType>(task.dynamics.type);
        if(!task_manager_.addTask(Task::makeTask(id, task.priority, task_type, task.sign, t_objs, TaskDynamics::makeTaskDynamics(dynamics_type,data))))
        {
            res.success = false;
            return res.success;
        }
        res.ids.push_back(id);
    }

    lock_.unlock();
    res.success = true;
    return res.success;
}
//------------------------------------------------------------------------
bool HQPVelocityController::setTaskObjects(hqp_controllers_msgs::SetTaskObjects::Request & req, hqp_controllers_msgs::SetTaskObjects::Response &res)
{
    lock_.lock();

    for(unsigned int i=0; i<req.objs.size();i++)
    {
        std::string root = req.objs[i].root;
        std::string link = req.objs[i].link;

        //try to get the assoicated kinematic chain
        boost::shared_ptr<KDL::Chain> chain(new KDL::Chain);

        if(!task_manager_.getKinematicTree()->getChain(root,link,(*chain)))
        {
            ROS_ERROR("Could not get kinematic chain from %s to %s.", root.c_str(),link.c_str());
            res.success =false;
            lock_.unlock();
            return res.success;
        }

        unsigned int id = task_manager_.getValidTaskObjectId();
        boost::shared_ptr<TaskObject> t_obj(new TaskObject(id, chain,root,joints_)); //create a new task object
        //parse the object geometries
        for (unsigned int j=0; j<req.objs[i].geometries.size();j++)
        {
            unsigned int n_data = req.objs[i].geometries[j].data.size();
            Eigen::VectorXd link_data(n_data);
            for (unsigned int k=0; k<n_data; k++)
                link_data(k)=req.objs[i].geometries[j].data[k];

            TaskGeometryType type = static_cast<TaskGeometryType>(req.objs[i].geometries[j].type);
            boost::shared_ptr<TaskGeometry> geom = TaskGeometry::makeTaskGeometry(type, link, root, link_data);
            t_obj->addGeometry(geom); //add the task geometries to the task object
        }
        t_obj->computeKinematics();
        task_manager_.addTaskObject(t_obj);
        res.ids.push_back(id);
    }
    lock_.unlock();
    res.success = true;
    return res.success;
}
//-----------------------------------------------------------------------
bool HQPVelocityController::visualizeTaskObjects(hqp_controllers_msgs::VisualizeTaskObjects::Request & req, hqp_controllers_msgs::VisualizeTaskObjects::Response &res)
{
    lock_.lock();
    vis_ids_.resize(req.ids.size());
    for(unsigned int i=0; i<req.ids.size();i++)
        vis_ids_(i)=req.ids[i];

    // populate the message
    vis_t_obj_pub_.msg_.markers.clear();
    if(task_manager_.getTaskGeometryMarkers(vis_t_obj_pub_.msg_,vis_ids_))
        res.success=true;
    else
        res.success=false;

    lock_.unlock();

    return res.success;
}
//-----------------------------------------------------------------------
void HQPVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
    task_manager_.computeTaskObjectsKinematics(); //compute jacobians and poses of the task objects

    if(active_)
    {
        //compute the HQP controls
        task_manager_.computeHQP();
        //set the computed task velocities if the computation was succesful, otherwise set them to zero
        if(!task_manager_.getDQ(commands_))
            commands_.setZero();

        //        std::cout<<"commands: "<<commands_.transpose()<<std::endl;
    }
    else
        commands_.setZero();

    for(unsigned int i=0; i<n_joints_; i++)
        joints_->at(i).setCommand(commands_(i));

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
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
    {
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // try to publish the task object geometries
        if (vis_t_obj_pub_.trylock())
            vis_t_obj_pub_.unlockAndPublish();

        // try to publish the task statuses
        task_manager_.getTaskStatuses(t_statuses_pub_.msg_);

        if (t_statuses_pub_.trylock())
            t_statuses_pub_.unlockAndPublish();

    }
    //======================= END PUBLISH =================
}
//-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPVelocityController,controller_interface::ControllerBase)
