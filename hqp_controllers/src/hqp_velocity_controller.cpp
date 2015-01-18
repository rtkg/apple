#include <hqp_controllers/hqp_velocity_controller.h>
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
    boost::shared_ptr<std::vector<boost::shared_ptr<TaskObject> > > t_obj_list(new std::vector<boost::shared_ptr<TaskObject> >); //this vector collects all the task objects loaded from the parameter server
    if (n.searchParam("collision_objects", searched_param))
    {
        n.getParam(searched_param,collision_objects);
        for (int32_t i = 0; i < collision_objects.size(); ++i)
        {
            std::string frame = (std::string)collision_objects[i]["frame"];
            boost::shared_ptr<KDL::Chain> chain(new KDL::Chain);
            if(!k_tree->getChain(collision_root,frame,(*chain)))
            {
                ROS_ERROR("Could not get kinematic chain from %s to %s.", collision_root.c_str(),frame.c_str());
                return false;
            }

            boost::shared_ptr<TaskObject> t_obj(new TaskObject(chain,joints_)); //create a new task object
            unsigned int m = (unsigned int)collision_objects[i]["geometries"].size();
            ROS_ASSERT(m >= 1); //make sure there is at least one task geometry associated with the newly created task object

            //iterate through the geometries corresponding to the present collision object
            for (int32_t j = 0; j < m ;j++)
            {
                boost::shared_ptr<TaskGeometry> t_geom;
                std::string type = (std::string)collision_objects[i]["geometries"][j]["type"];
                if (type == "POINT")
                {
                    //read the point's position
                    boost::shared_ptr<Eigen::Vector3d> p(new Eigen::Vector3d);
                    for (int32_t k = 0; k < 3; k++)
                        (*p)(k)= (double)collision_objects[i]["geometries"][j]["p"][k];

                    t_geom.reset(new Point(frame,p));
                }
                else if (type == "PLANE")
                {
                    //read normal and offset of the plane
                    boost::shared_ptr<Eigen::Vector3d> n(new Eigen::Vector3d);
                    for (int32_t k = 0; k < 3 ;k++)
                        (*n)(k) = (double)collision_objects[i]["geometries"][j]["n"][k];

                    double d = (double)collision_objects[i]["geometries"][j]["d"];
                    d=d/n->norm(); n->normalize(); //normalize the plane quantities

                    t_geom.reset(new Plane(frame,n,d));
                }
                else if (type == "CAPSULE")
                {
                    //read the capsule's start vector, end vector and radius
                    boost::shared_ptr<Eigen::Vector3d> p(new Eigen::Vector3d);
                    boost::shared_ptr<Eigen::Vector3d> t(new Eigen::Vector3d);
                    for (int32_t k = 0; k < 3 ;k++)
                    {
                        (*p)(k) = (double)collision_objects[i]["geometries"][j]["p"][k];
                        (*t)(k) = (double)collision_objects[i]["geometries"][j]["t"][k];
                    }
                    double r = (double)collision_objects[i]["geometries"][j]["r"];

                    t_geom.reset(new Capsule(frame,p,t,r));
                }
                else
                {
                    ROS_ERROR("Invalid collision geometry type!");
                    return false;
                }
                t_obj->addGeometry(t_geom); //add the task geometries to the task object
            }
            t_obj_list->push_back(t_obj);
        }
        ROS_INFO("Collision objects loaded");
    }
    else
    {
        ROS_WARN("No collsion objects specified!");
    }

    task_manager_.initialize(k_tree,t_obj_list);
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
//-----------------------------------------------------------------------
} //end namespace hqp_controllers

PLUGINLIB_EXPORT_CLASS(hqp_controllers::HQPVelocityController,controller_interface::ControllerBase)
