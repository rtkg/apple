#include <hqp_controllers/hqp_velocity_controller.h>
#include <hqp_controllers/task_geometry.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Geometry>

namespace hqp_controllers
{
//-----------------------------------------------------------------------
HQPVelocityController::HQPVelocityController()
{
    joints_.reset(new std::vector< hardware_interface::JointHandle >);
    commands_.clear();
}
//-----------------------------------------------------------------------
HQPVelocityController::~HQPVelocityController()
{
    sub_command_.shutdown();
    set_task_obj_srv_.shutdown();
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
            task_manager_.addTaskObject(t_obj); //store the new object in the task manager
        }
        ROS_INFO("Collision objects loaded");
    }
    else
    {
        ROS_WARN("No collsion objects specified!");
    }

    //============================================== REGISTER CALLBACKS =========================================
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &HQPVelocityController::commandCB, this);
    set_task_obj_srv_ = n.advertiseService("set_task_object",&HQPVelocityController::setTaskObject,this);
    //============================================== REGISTER CALLBACKS END =========================================

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
    task_manager_.computeTaskObjectsKinematics(); //compute jacobians and poses of the task objects

    for(unsigned int i=0; i<n_joints_; i++)
        joints_->at(i).setCommand(commands_[i]);

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
}


///////////////
// CALLBACKS //
///////////////

//-----------------------------------------------------------------------
void HQPVelocityController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size()!=n_joints_)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    for(unsigned int i=0; i<n_joints_; i++)
        commands_[i] = msg->data[i];
}
//------------------------------------------------------------------------
bool HQPVelocityController::setTaskObject(hqp_controllers_msgs::SetTaskObject::Request & req, hqp_controllers_msgs::SetTaskObject::Response &res)
{
    std::string root = req.obj.root;
    std::string link = req.obj.link;

    //try to get the assoicated kinematic chain
    boost::shared_ptr<KDL::Chain> chain(new KDL::Chain);
    lock_.lock();
    if(!task_manager_.getKinematicTree()->getChain(root,link,(*chain)))
    {
        ROS_ERROR("Could not get kinematic chain from %s to %s.", root.c_str(),link.c_str());
        res.success =false;
        lock_.unlock();
        return res.success;
    }

    boost::shared_ptr<TaskObject> t_obj(new TaskObject(task_manager_.getValidTaskObjectId(),chain,root,joints_)); //create a new task object
    //parse the object geometries
    for (unsigned int i=0; i<req.obj.geometries.size();i++)
    {
        unsigned int n_data = req.obj.geometries[i].data.size();
        Eigen::VectorXd link_data(n_data);
        for (unsigned int j=0; j<n_data; j++)
            link_data(j)=req.obj.geometries[i].data[j];

        TaskGeometryType type = static_cast<TaskGeometryType>(req.obj.geometries[i].type);
        boost::shared_ptr<TaskGeometry> geom = TaskGeometry::makeTaskGeometry(type, link, root, link_data);
        t_obj->addGeometry(geom); //add the task geometries to the task object
    }

    task_manager_.addTaskObject(t_obj);
    lock_.unlock();

    res.success = true;
    return res.success;
}
//-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPVelocityController,controller_interface::ControllerBase)
