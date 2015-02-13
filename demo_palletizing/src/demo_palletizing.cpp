#include <demo_palletizing/demo_palletizing.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <math.h>
#include <limits>

#include <hqp_controllers_msgs/TaskGeometry.h>
#include <hqp_controllers_msgs/RemoveTasks.h>
#include <hqp_controllers_msgs/RemoveTaskObjects.h>
#include <hqp_controllers_msgs/ActivateHQPControl.h>
#include <hqp_controllers_msgs/VisualizeTaskObjects.h>

namespace demo_palletizing
{
//-----------------------------------------------------------------
DemoPalletizing::DemoPalletizing() : task_error_tol_(0.0)
{

    //handle to home
    nh_ = ros::NodeHandle("~");
    //global handle
    n_ = ros::NodeHandle();

    //get params
    bool with_gazebo;
    nh_.param<bool>("with_gazebo", with_gazebo,false);
    if(with_gazebo)
        ROS_INFO("Demo palletizing running in Gazebo.");

    //initialize variables
    task_status_changed_ = false;
    task_success_ = false;

    //register callbacks
    start_demo_srv_ = nh_.advertiseService("start_demo", &DemoPalletizing::startDemo, this);;
    task_status_sub_ = n_.subscribe("task_statuses", 1, &DemoPalletizing::stateCallback, this);
    set_tasks_clt_ = n_.serviceClient<hqp_controllers_msgs::SetTasks>("set_tasks");
    set_task_objects_clt_ = n_.serviceClient<hqp_controllers_msgs::SetTaskObjects>("set_task_objects");
    remove_task_clt_ = n_.serviceClient<hqp_controllers_msgs::RemoveTasks>("remove_tasks");
    remove_task_objects_clt_ = n_.serviceClient<hqp_controllers_msgs::RemoveTaskObjects>("remove_task_objects");
    activate_hqp_control_clt_ = n_.serviceClient<hqp_controllers_msgs::ActivateHQPControl>("activate_hqp_control");
    visualize_task_objects_clt_ = n_.serviceClient<hqp_controllers_msgs::VisualizeTaskObjects>("visualize_task_objects");
    set_gazebo_physics_clt_ = n_.serviceClient<gazebo_msgs::SetPhysicsProperties>("set_physics_properties");

    set_tasks_clt_.waitForExistence();
    set_task_objects_clt_.waitForExistence();
    remove_task_clt_.waitForExistence();
    remove_task_objects_clt_.waitForExistence();
    activate_hqp_control_clt_.waitForExistence();
    visualize_task_objects_clt_.waitForExistence();

    //if gazebo is used, set the simulated gravity to zero in order to prevent gazebo's joint drifting glitch
    if(with_gazebo)
    {
        set_gazebo_physics_clt_.waitForExistence();

        gazebo_msgs::SetPhysicsProperties properties;
        properties.request.time_step = 0.001;
        properties.request.max_update_rate = 1000;
        properties.request.gravity.x = 0.0;
        properties.request.gravity.y = 0.0;
        properties.request.gravity.z = 0.0;
        properties.request.ode_config.auto_disable_bodies = false;
        properties.request.ode_config. sor_pgs_precon_iters = 0;
        properties.request.ode_config. sor_pgs_iters = 50;
        properties.request.ode_config. sor_pgs_w = 1.3;
        properties.request.ode_config.sor_pgs_rms_error_tol = 0.0;
        properties.request.ode_config.contact_surface_layer = 0.001;
        properties.request.ode_config.contact_max_correcting_vel = 100.0;
        properties.request.ode_config.cfm = 0.0;
        properties.request.ode_config.erp = 0.2;
        properties.request.ode_config.max_contacts= 20.0;

        set_gazebo_physics_clt_.call(properties);
        if(!properties.response.success)
        {
            ROS_ERROR("Couldn't set Gazebo physics properties, status message: %s!", properties.response.status_message.c_str());
            ros::shutdown();
        }
        else
            ROS_INFO("Disabled gravity in Gazebo.");

    }

    //configs have to be within the safety margins of the joint limits
#ifdef HQP_GRIPPER_JOINT
    unsigned int n_jnts = 8;
#else
    unsigned int n_jnts = 7;
#endif

    home_config_ = std::vector<double>(n_jnts, 0.1); //7 arm joint + 1 velvet fingers gripper joint
    transfer_config_ = std::vector<double>(n_jnts);
    transfer_config_[0] = 0;
    transfer_config_[1] = -1.57;
    transfer_config_[2] = 2.42;
    transfer_config_[3] = -1.0;
    transfer_config_[4] = 0;
    transfer_config_[5] = 0.52;
    transfer_config_[6] = 0.0;
#ifdef HQP_GRIPPER_JOINT
    transfer_config_[7] = 0.1;
#endif

    sensing_config_ = std::vector<double>(n_jnts);
    // sensing_config_[0] = 1.48;
    // sensing_config_[1] = -1.2;
    // sensing_config_[2] = 0.19;
    // sensing_config_[3] = -1.92;
    // sensing_config_[4] = -1.45;
    // sensing_config_[5] = 0.79;
    // sensing_config_[6] = 0;

    sensing_config_[0] = 1.27;
    sensing_config_[1] = -1.9;
    sensing_config_[2] = -0.52;
    sensing_config_[3] = -1.9;
    sensing_config_[4] = -0.46;
    sensing_config_[5] = 0.79;
    sensing_config_[6] = -1.48;
#ifdef HQP_GRIPPER_JOINT
    sensing_config_[7] = 0.1;
#endif

    //Grasp intervall specification
    grasp_.obj_frame_ = "load_pallet_base";  //object frame
    grasp_.e_frame_ = "velvet_fingers_palm"; //endeffector frame
    grasp_.e_.setZero();                     //endeffector point expressed in the endeffector frame
    grasp_.v_(0) = 0.0; grasp_.v_(1) = 0.0; grasp_.v_(2) = 1.0; //cylinder normal
    grasp_.p_(0) = 0.0; grasp_.p_(1) = 0.0; grasp_.p_(2) = 0.13; //reference point on the cylinder axis
    grasp_.r1_ = 0.1; grasp_.r2_ = 0.2;              //cylinder radii
    grasp_.n1_ = grasp_.v_; grasp_.n2_ = grasp_.v_;  //plane normals
    grasp_.d1_ = 0.25; grasp_.d2_= 0.35;              //plane offsets

    //generate the task object templates
    generateTaskObjectTemplates();
}
//-----------------------------------------------------------------
void DemoPalletizing::generateTaskObjectTemplates()
{
    hqp_controllers_msgs::TaskObject t_obj;
    hqp_controllers_msgs::TaskGeometry t_geom;
    std::vector<double> data;

    //endeffector point
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.e_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::POINT;
    data.resize(3);
    data[0] = grasp_.e_(0); data[1] = grasp_.e_(1); data[2] = grasp_.e_(2);
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["ee_point"] = t_obj;

    //inner grasp cylinder
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    data.resize(7);
    data[0] = grasp_.p_(0); data[1] = grasp_.p_(1); data[2] = grasp_.p_(2);
    data[3] = grasp_.v_(0); data[4] = grasp_.v_(1); data[5] = grasp_.v_(2);
    data[6] = grasp_.r1_;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["inner_bound_cylinder"] = t_obj;

    //outer grasp cylinder
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    data.resize(7);
    data[0] = grasp_.p_(0); data[1] = grasp_.p_(1); data[2] = grasp_.p_(2);
    data[3] = grasp_.v_(0); data[4] = grasp_.v_(1); data[5] = grasp_.v_(2);
    data[6] = grasp_.r2_;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["outer_bound_cylinder"] = t_obj;

    //upper grasp plane
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::PLANE;
    data.resize(4);
    data[0] = grasp_.n1_(0); data[1] = grasp_.n1_(1); data[2] = grasp_.n1_(2);
    data[3] = grasp_.d1_;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["upper_bound_plane"] = t_obj;

    //lower grasp plane
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::PLANE;
    data.resize(4);
    data[0] = grasp_.n2_(0); data[1] = grasp_.n2_(1); data[2] = grasp_.n2_(2);
    data[3] = grasp_.d2_;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lower_bound_plane"] = t_obj;

    //vertical gripper axis
    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "velvet_fingers_palm";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::LINE;
    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction
    data.resize(6);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.0; data[3] = 0.0; data[5] = 1.0;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["gripper_vertical_axis"] = t_obj;

    //gripper approach axis
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.e_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::LINE;
    data.resize(6);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 1.0; data[4] = 0.0; data[5] = 0.0;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["gripper_approach_axis"] = t_obj;

    //inner constraint cone for the vertical gripper axis
    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "world";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::CONE;
    //first 3 entries of link_data are the cone's reference point, the second 3 entries the direction, the last entry is the opening angle
    data.resize(7);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.0; data[4] = 0.0; data[5] = 1.0; //gripper axis should be vertical
    data[6] = 0.035; //corresponds to an angle of 2 degree
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["upper_bound_cone"] = t_obj;

    //coplanar target line
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::LINE;
    data.resize(6);
    data[0] = grasp_.p_(0); data[1] = grasp_.p_(1); data[2] = grasp_.p_(2);
    data[3] = grasp_.v_(0); data[4] = grasp_.v_(1); data[5] = grasp_.v_(2);
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["coplanar_target_line"] = t_obj;

    //extract plane
    t_obj.geometries.clear();
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::PLANE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 1.0;
    data[3] = 0.55;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["extract_bound_plane"] = t_obj;

    //placement plane
    t_obj.geometries.clear();
    t_obj.root = "transport_pallet_base";
    t_obj.link = "transport_pallet_base";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::PLANE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 1.0;
    data[3] = 0.3;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["placement_bound_plane"] = t_obj;

    //placement cylinder
    t_obj.geometries.clear();
    t_obj.root = "transport_pallet_base";
    t_obj.link = "transport_pallet_base";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    data.resize(7);
    data[0] = 0.1; /*0.15*/ data[1] = -0.15; /*-0.2*/ data[2] = 0.13;
    data[3] = 0.0; data[4] = 0.0; data[5] = 1.0;
    data[6] = 0.05;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["placement_bound_cylinder"] = t_obj;

    //joint position targets
    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_0";
    t_obj.link = "lbr_iiwa_link_1";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_1_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_1";
    t_obj.link = "lbr_iiwa_link_2";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_2_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_2";
    t_obj.link = "lbr_iiwa_link_3";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_3_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_3";
    t_obj.link = "lbr_iiwa_link_4";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_4_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_4";
    t_obj.link = "lbr_iiwa_link_5";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_5_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_5";
    t_obj.link = "lbr_iiwa_link_6";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_6_target"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "lbr_iiwa_link_6";
    t_obj.link = "lbr_iiwa_link_7";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_joint_7_target"] = t_obj;

#ifdef HQP_GRIPPER_JOINT
    t_obj.geometries.clear();
    t_obj.root = "velvet_fingers_palm";
    t_obj.link = "velvet_fingers_right";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;
    t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["velvet_fingers_joint_1_target"] = t_obj;
#endif

    //collision planes
    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "world";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::PLANE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 1.0;
    data[3] = 0.16;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["collision_planes"] = t_obj;

    //collision spheres
    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "lbr_iiwa_link_3";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.075;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_link_3_sphere"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "lbr_iiwa_link_4";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.12;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_link_4_sphere"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "lbr_iiwa_link_5";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.075;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_link_5_sphere"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "lbr_iiwa_link_6";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    data.resize(4);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.1;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["lbr_iiwa_link_6_sphere"] = t_obj;

    t_obj.geometries.clear();
    t_obj.root = "world";
    t_obj.link = "velvet_fingers_palm";
    t_geom.type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    data.resize(4);
    data[0] = 0.04; data[1] = 0.0; data[2] = 0.025;
    data[3] = 0.11;
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_object_templates_["velvet_fingers_palm_sphere"] = t_obj;
}
//-----------------------------------------------------------------
void DemoPalletizing::activateHQPControl()
{
    hqp_controllers_msgs::ActivateHQPControl controller_status;
    controller_status.request.active = true;
    activate_hqp_control_clt_.call(controller_status);
}
//-----------------------------------------------------------------
void DemoPalletizing::deactivateHQPControl()
{
    hqp_controllers_msgs::ActivateHQPControl controller_status;
    controller_status.request.active = false;
    activate_hqp_control_clt_.call(controller_status);
}
//-----------------------------------------------------------------
void DemoPalletizing::safeShutdown()
{
    deactivateHQPControl();
    ROS_BREAK(); //I must break you ... ros::shutdown() doesn't seem to do the job
}
//-----------------------------------------------------------------
bool DemoPalletizing::resetState()
{
    //send an empty visualization message to stop visualizing the current task objects
    hqp_controllers_msgs::VisualizeTaskObjects vis_srv;
    visualize_task_objects_clt_.call(vis_srv);
    if(!vis_srv.response.success)
    {
        ROS_ERROR("DemoPalletizing::resetStateTasks(): could not stop visualization!");
        return false;
    }

    hqp_controllers_msgs::RemoveTaskObjects rem_t_obj_srv;
    for(unsigned int i=0; i<task_objects_.response.ids.size();i++)
        rem_t_obj_srv.request.ids.push_back(task_objects_.response.ids[i]);

    remove_task_objects_clt_.call(rem_t_obj_srv);
    if(!rem_t_obj_srv.response.success)
    {
        ROS_ERROR("DemoPalletizing::resetStateTasks(): could not remove task objects!");
        return false;
    }
    //clean up the task object message which is used as a container
    task_objects_.response.ids.clear();
    task_objects_.response.success = false;
    task_objects_.request.objs.clear();

    hqp_controllers_msgs::RemoveTasks rem_t_srv;
    for(unsigned int i=0; i<tasks_.response.ids.size(); i++)
        rem_t_srv.request.ids.push_back(tasks_.response.ids[i]);

    remove_task_clt_.call(rem_t_srv);
    if(!rem_t_srv.response.success)
    {
        ROS_ERROR("DemoPalletizing::resetStateTasks(): could not remove tasks!");
        return false;
    }
    //clean up the task message which is used as a container
    tasks_.response.ids.clear();
    tasks_.response.success = false;
    tasks_.request.tasks.clear();

    //clean up the monitored tasks
    monitored_tasks_.clear();

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::visualizeStateTaskObjects(std::vector<unsigned int> const& ids)
{
    //send a visualization message to show the task objects in Rviz
    hqp_controllers_msgs::VisualizeTaskObjects vis_srv;
    for(unsigned int i=0; i<ids.size();i++)
        vis_srv.request.ids.push_back(ids[i]);

    visualize_task_objects_clt_.call(vis_srv);
    if(!vis_srv.response.success)
    {
        ROS_ERROR("DemoPalletizing::setStateTasks(): could not start visualization!");
        return false;
    }
    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::sendStateTasks()
{
    //sends the tasks to the controller
    set_tasks_clt_.call(tasks_);
    if(!tasks_.response.success)
    {
        ROS_ERROR("DemoPalletizing::setStateTasks(): could not set tasks!");
        return false;
    }
    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::sendStateTaskObjects()
{
    //sends the task objects to the controller
    set_task_objects_clt_.call(task_objects_);
    if(!task_objects_.response.success)
    {
        ROS_ERROR("DemoPalletizing::setStateTaskObjects(): could not set task objects!");
        return false;
    }
    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setObjectTransfer()
{
    //TASK OBJECTS
    hqp_controllers_msgs::TaskObject t_obj;
    t_obj = task_object_templates_["ee_point"];
    t_obj.link = "velvet_fingers_palm";
    t_obj.root = "transport_pallet_base";
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["placement_bound_plane"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["placement_bound_cylinder"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["upper_bound_cone"];
    t_obj.geometries[0].data[6] = 1e-3; //reduce the tilting angle of the vertical axis
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["gripper_vertical_axis"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["collision_planes"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_3_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_4_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_5_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_6_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["velvet_fingers_palm_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_1_target"];
    t_obj.geometries[0].data[0] = 0.57;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_2_target"];
    t_obj.geometries[0].data[0] = 0.54;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_3_target"];
    t_obj.geometries[0].data[0] = 0.64;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_4_target"];
    t_obj.geometries[0].data[0] = -1.56;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_5_target"];
    t_obj.geometries[0].data[0] = -0.26;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_6_target"];
    t_obj.geometries[0].data[0] = 0.99;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_7_target"];
    t_obj.geometries[0].data[0] = -1.91;
    task_objects_.request.objs.push_back(t_obj);

    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    //TASKS
    hqp_controllers_msgs::Task task;
    //bring endeffector on the placement plane
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_PLANE;
    task.priority = 2;
    task.sign = "=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[1]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //bring endeffector point inside the placement cylinder
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_CYLINDER;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[2]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep vertical axis of the gripper inside the vertical cone
    task.type = hqp_controllers_msgs::Task::ANGLE_LINES;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[3]);
    task.t_obj_ids.push_back(task_objects_.response.ids[4]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the avoidance tasks
    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[6]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[7]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[8]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[10]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the joint setpoint tasks
    for(unsigned int i=11; i<18; i++)
    {
        task.type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
        task.priority = 3;
        task.sign = "=";
        task.t_obj_ids.clear();
        task.t_obj_ids.push_back(task_objects_.response.ids[i]);
        std::cout<<"using object nr: "<<i<<std::endl;
        task.dynamics.data.clear();
        task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
        task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
        tasks_.request.tasks.push_back(task);
    }

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //don't monitor the joint position reference tasks
    for(unsigned int i=0; i<8; i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize the task objects
    std::vector<unsigned int> ids;
    for(unsigned int i=1; i<5; i++)
        ids.push_back(task_objects_.response.ids[i]);

    if(!visualizeStateTaskObjects(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setObjectExtract()
{
    //TASK OBJECTS
    hqp_controllers_msgs::TaskObject t_obj;
    t_obj = task_object_templates_["ee_point"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["extract_bound_plane"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["outer_bound_cylinder"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["upper_bound_cone"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["gripper_vertical_axis"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["collision_planes"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_3_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_4_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_5_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_6_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["velvet_fingers_palm_sphere"];
    task_objects_.request.objs.push_back(t_obj);


    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    //TASKS
    hqp_controllers_msgs::Task task;
    //bring endeffector on the extract plane
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_PLANE;
    task.priority = 2;
    task.sign = "=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[1]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep endeffector point inside the second cylinder
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_CYLINDER;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[2]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep vertical axis of the gripper inside the vertical cone
    task.type = hqp_controllers_msgs::Task::ANGLE_LINES;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[3]);
    task.t_obj_ids.push_back(task_objects_.response.ids[4]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the avoidance tasks
    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[6]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[7]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[8]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[10]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize the task objects
    std::vector<unsigned int> ids;
    for(unsigned int i=1; i<5; i++)
        ids.push_back(task_objects_.response.ids[i]);

    if(!visualizeStateTaskObjects(ids))
        return false;

    return true;
}
//---------------------------------------------------------------------
bool DemoPalletizing::setGripperExtract()
{
    //TASK OBJECTS
    hqp_controllers_msgs::TaskObject t_obj;
    t_obj = task_object_templates_["ee_point"];
    t_obj.link = "velvet_fingers_palm";
    t_obj.root = "transport_pallet_base";
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["extract_bound_plane"];
    t_obj.link = "transport_pallet_base";
    t_obj.root = "transport_pallet_base";
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["placement_bound_cylinder"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["upper_bound_cone"];
    t_obj.geometries[0].data[6] = 1e-3; //reduce the tilting angle of the vertical axis
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["gripper_vertical_axis"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["collision_planes"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_3_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_4_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_5_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_6_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["velvet_fingers_palm_sphere"];
    task_objects_.request.objs.push_back(t_obj);


    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    //TASKS
    hqp_controllers_msgs::Task task;
    //bring endeffector on the extract plane
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_PLANE;
    task.priority = 2;
    task.sign = "=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[1]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep endeffector point inside the placement cylinder
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_CYLINDER;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[2]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep vertical axis of the gripper inside the vertical cone
    task.type = hqp_controllers_msgs::Task::ANGLE_LINES;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[3]);
    task.t_obj_ids.push_back(task_objects_.response.ids[4]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the avoidance tasks
    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[6]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[7]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[8]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[10]);
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize the task objects
    std::vector<unsigned int> ids;
    for(unsigned int i=0; i<task_objects_.response.ids.size(); i++)
        ids.push_back(task_objects_.response.ids[i]);

    if(!visualizeStateTaskObjects(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setGraspApproach()
{
    ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);

    //TASK OBJECTS
    hqp_controllers_msgs::TaskObject t_obj;
    t_obj = task_object_templates_["ee_point"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["inner_bound_cylinder"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["outer_bound_cylinder"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["upper_bound_plane"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lower_bound_plane"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["upper_bound_cone"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["gripper_vertical_axis"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["coplanar_target_line"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["gripper_approach_axis"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["collision_planes"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_3_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_4_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_5_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_6_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["velvet_fingers_palm_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    //TASKS
    //keep endeffector point outside the first cylinder
    hqp_controllers_msgs::Task task;
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_CYLINDER;
    task.priority = 2;
    task.sign = ">=";
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[1]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //bring endeffector point inside the second cylinder
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_CYLINDER;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[2]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep endeffector point above the first plane
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_PLANE;
    task.priority = 2;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[3]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep endeffector point below the second plane
    task.type = hqp_controllers_msgs::Task::PROJECT_POINT_PLANE;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.t_obj_ids.push_back(task_objects_.response.ids[4]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //keep vertical axis of the gripper inside the vertical cone
    task.type = hqp_controllers_msgs::Task::ANGLE_LINES;
    task.priority = 2;
    task.sign = "<=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.t_obj_ids.push_back(task_objects_.response.ids[6]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //    //keep the gripper's approach axis coplanar with the corresponding line (which runs along the object axis)
    //    task.type = hqp_controllers_msgs::Task::COPLANAR_LINES;
    //    task.priority = 2;
    //    task.sign = "=";
    //    task.t_obj_ids.clear();
    //    task.t_obj_ids.push_back(task_objects_.response.ids[7]);
    //    task.t_obj_ids.push_back(task_objects_.response.ids[8]);
    //    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    //    task.dynamics.data.clear();
    //    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    //    tasks_.request.tasks.push_back(task);

    //second variant of keep the gripper's approach axis coplanar with the corresponding line (which runs along the object axis)
    task.type = hqp_controllers_msgs::Task::PROJECT_LINE_LINE;
    task.priority = 2;
    task.sign = "=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[7]);
    task.t_obj_ids.push_back(task_objects_.response.ids[8]);
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.clear();
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the avoidance tasks
    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[10]);
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[11]);
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[12]);
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[13]);
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[14]);
    task.t_obj_ids.push_back(task_objects_.response.ids[9]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize task objects
    std::vector<unsigned int> ids;
    for (unsigned int i=2; i<9; i++)
        ids.push_back(task_objects_.response.ids[i]);

    if(!visualizeStateTaskObjects(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setJointConfiguration(std::vector<double> const& joints)
{
#ifdef HQP_GRIPPER_JOINT
    ROS_ASSERT(joints.size() == 8);//7 joints for the lbr iiwa + 1 velvet fingers joint
#else
    ROS_ASSERT(joints.size() == 7);
#endif

    //fill in the task_objects_
    hqp_controllers_msgs::TaskObject t_obj;
    t_obj = task_object_templates_["collision_planes"];
    t_obj.geometries[0].data[3] = 0.3;
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_3_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_4_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_5_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_link_6_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["velvet_fingers_palm_sphere"];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_1_target"];
    t_obj.geometries[0].data[0] = joints[0];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_2_target"];
    t_obj.geometries[0].data[0] = joints[1];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_3_target"];
    t_obj.geometries[0].data[0] = joints[2];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_4_target"];
    t_obj.geometries[0].data[0] = joints[3];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_5_target"];
    t_obj.geometries[0].data[0] = joints[4];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_6_target"];
    t_obj.geometries[0].data[0] = joints[5];
    task_objects_.request.objs.push_back(t_obj);

    t_obj = task_object_templates_["lbr_iiwa_joint_7_target"];
    t_obj.geometries[0].data[0] = joints[6];
    task_objects_.request.objs.push_back(t_obj);

#ifdef HQP_GRIPPER_JOINT
    t_obj = task_object_templates_["velvet_fingers_joint_1_target"];
    t_obj.geometries[0].data[0] = joints[7];
    task_objects_.request.objs.push_back(t_obj);
#endif

    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    hqp_controllers_msgs::Task task;
    //fill in the avoidance tasks
    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[1]);
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[2]);
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[3]);
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[4]);
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    task.type = hqp_controllers_msgs::Task::PROJECT_SPHERE_PLANE;
    task.priority = 1;
    task.sign = ">=";
    task.t_obj_ids.clear();
    task.t_obj_ids.push_back(task_objects_.response.ids[5]);
    task.t_obj_ids.push_back(task_objects_.response.ids[0]);
    task.dynamics.data.clear();
    task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
    tasks_.request.tasks.push_back(task);

    //fill in the joint setpoint tasks
    for(unsigned int i=6; i<joints.size()+6; i++)
    {
        task.type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
        task.priority = 2;
        task.sign = "=";
        task.t_obj_ids.clear();
        task.t_obj_ids.push_back(task_objects_.response.ids[i]);
        task.dynamics.data.clear();
        task.dynamics.type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
        task.dynamics.data.push_back(TASK_DYNAMICS_GAIN);
        tasks_.request.tasks.push_back(task);
    }

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //all joint tasks will be monitored, fill in the ids obtained from the response of the previously exectued service call
    for(unsigned int i=0; i<joints.size(); i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //could start joint target visualization here, but its messed up for the joints anyway ...
    //visualize the collision objects
//    std::vector<unsigned int> ids;
//    ids.push_back(task_objects_.response.ids[0]);
//    ids.push_back(task_objects_.response.ids[1]);
//    ids.push_back(task_objects_.response.ids[2]);
//    ids.push_back(task_objects_.response.ids[3]);
//    ids.push_back(task_objects_.response.ids[4]);
//    ids.push_back(task_objects_.response.ids[5]);
//    if(!visualizeStateTaskObjects(ids))
//        return false;

    return true;
}
//-----------------------------------------------------------------
void DemoPalletizing::stateCallback( const hqp_controllers_msgs::TaskStatusesPtr& msg)
{
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);

    //    std::cout<<"monitor tasks: ";
    //      for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    //          std::cout<<monitored_tasks_[i]<<" ";

    //      std::cout<<std::endl;

    //form the maximum norm over all errors
    double e = 0.0;
    for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    {
        //try to find the monitored task id in the given task status message
        std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it;
        for(status_it = msg->statuses.begin(); status_it!=msg->statuses.end(); ++status_it)
            if(monitored_tasks_[i] == status_it->id)
            {
                //std::cout<<"task id: "<<status_it->id<<" sse: "<<status_it->sse<<std::endl;
                //found the corresponding task in the status message
                if (status_it->sse > e)
                    e = status_it->sse;

                break;
            }

        if(status_it==msg->statuses.end())
        {
            ROS_WARN("No status feedback for monitored task id %d!", monitored_tasks_[i]);
            return; //just so we don't give a false positive task success
        }

    }

    if(e <= task_error_tol_)
    {
        std::cout<<std::endl<<"STATE CHANGE:"<<std::endl<<"monitored tasks: ";
        for(unsigned int i=0; i<monitored_tasks_.size(); i++)
            std::cout<<monitored_tasks_[i]<<" ";

        std::cout<<std::endl<<"task statuses: "<<std::endl;
        for( std::vector<hqp_controllers_msgs::TaskStatus>::iterator it = msg->statuses.begin(); it!=msg->statuses.end(); ++it)
            std::cout<<"id: "<<it->id<<" sse: "<<it->sse<<std::endl;

        std::cout<<"e: "<<e<<std::endl<<std::endl;

        // ROS_INFO("Task status switch!");
        task_status_changed_ = true;
        task_success_ = true;
        cond_.notify_one();
    }
    //        else if //(should detect task timeout here ...)
    //    {
    //        task_status_changed_ = true;
    //        task_success_ = false;
    //        ROS_ERROR("Task execution timeout!");
    //        cond_.notify_one();
    //    }
}
//-----------------------------------------------------------------
bool DemoPalletizing::startDemo(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{

    //         {//MANIPULATOR HOME CONFIGURATION
    //             ROS_INFO("Trying to home the manipulator.");
    //             boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    //             task_status_changed_ = false;
    //             task_success_ = false;
    //             deactivateHQPControl(); //better safe than sorry ...
    //             if(!resetState())
    //             {
    //                 ROS_ERROR("Could not reset the state!");
    //                 safeShutdown();
    //                 return false;
    //             }
    //             if(!setJointConfiguration(home_config_))
    //             {
    //                 ROS_ERROR("Could not set manipulator home state!");
    //                 safeShutdown();
    //                 return false;
    //             }
    //             task_error_tol_ = 1e-2;
    //             activateHQPControl();

    //             while(!task_status_changed_)
    //                 cond_.wait(lock);

    //             if(!task_success_)
    //             {
    //                 ROS_ERROR("Could not complete the manipulator home state tasks!");
    //                 safeShutdown();
    //                 return false;
    //             }
    //             ROS_INFO("Manipulator home state tasks executed successfully.");
    //         }

//    {//MANIPULATOR TRANSFER CONFIGURATION
//        ROS_INFO("Trying to put the manipulator in transfer configuration.");
//        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
//        task_status_changed_ = false;
//        task_success_ = false;
//        deactivateHQPControl();
//        if(!resetState())
//        {
//            ROS_ERROR("Could not reset the state!");
//            safeShutdown();
//            return false;
//        }
//        if(!setJointConfiguration(transfer_config_))
//        {
//            ROS_ERROR("Could not set manipulator transfer state!");
//            safeShutdown();
//            return false;
//        }
//        task_error_tol_ = 1e-2;
//        activateHQPControl();

//        while(!task_status_changed_)
//            cond_.wait(lock);

//        if(!task_success_)
//        {
//            ROS_ERROR("Could not complete the manipulator transfer state tasks!");
//            safeShutdown();
//            return false;
//        }
//        ROS_INFO("Manipulator transfer state tasks executed successfully.");
//    }

    {//MANIPULATOR SENSING CONFIGURATION
        ROS_INFO("Trying to put the manipulator in sensing configuration.");
        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
        task_status_changed_ = false;
        task_success_ = false;
        deactivateHQPControl();
        if(!resetState())
        {
            ROS_ERROR("Could not reset the state!");
            safeShutdown();
            return false;
        }
        if(!setJointConfiguration(sensing_config_))
        {
            ROS_ERROR("Could not set manipulator sensing state!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the manipulator sensing state tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Manipulator sensing state tasks executed successfully.");
    }

    {//GRASP APPROACH
        ROS_INFO("Trying grasp approach.");
        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
        task_status_changed_ = false;
        task_success_ = false;
        deactivateHQPControl();
        if(!resetState())
        {
            ROS_ERROR("Could not reset the state!");
            safeShutdown();
            return false;
        }
        if(!setGraspApproach())
        {
            ROS_ERROR("Could not set the grasp approach!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-3;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the grasp approach tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Grasp approach tasks executed successfully.");
    }

    {//OBJECT EXTRACT
        ROS_INFO("Trying object extract.");
        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
        task_status_changed_ = false;
        task_success_ = false;
        deactivateHQPControl();
        if(!resetState())
        {
            ROS_ERROR("Could not reset the state!");
            safeShutdown();
            return false;
        }
        if(!setObjectExtract())
        {
            ROS_ERROR("Could not set the object extract!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the object extract tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Object extract tasks executed successfully.");
    }

    {//OBJECT TRANSFER
        ROS_INFO("Trying object transfer.");
        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
        task_status_changed_ = false;
        task_success_ = false;
        deactivateHQPControl();
        if(!resetState())
        {
            ROS_ERROR("Could not reset the state!");
            safeShutdown();
            return false;
        }
        if(!setObjectTransfer())
        {
            ROS_ERROR("Could not set the object transfer!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-3;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the object transfer tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Object transfer tasks executed successfully.");
    }

//    {//GRIPPER EXTRACT
//        ROS_INFO("Trying gripper extract.");
//        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
//        task_status_changed_ = false;
//        task_success_ = false;
//        deactivateHQPControl();
//        if(!resetState())
//        {
//            ROS_ERROR("Could not reset the state!");
//            safeShutdown();
//            return false;
//        }

//        if(!setGripperExtract())
//        {
//            ROS_ERROR("Could not set the gripper extract!");
//            safeShutdown();
//            return false;
//        }
//        task_error_tol_ = 1e-2;
//        activateHQPControl();

//        while(!task_status_changed_)
//            cond_.wait(lock);

//        if(!task_success_)
//        {
//            ROS_ERROR("Could not complete the gripper extract tasks!");
//            safeShutdown();
//            return false;
//        }
//        ROS_INFO("Gripper extract tasks executed successfully.");
//    }

//    {//MANIPULATOR TRANSFER CONFIGURATION
//        ROS_INFO("Trying to put the manipulator in transfer configuration.");
//        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
//        task_status_changed_ = false;
//        task_success_ = false;
//        deactivateHQPControl();
//        if(!resetState())
//        {
//            ROS_ERROR("Could not reset the state!");
//            safeShutdown();
//            return false;
//        }
//        if(!setJointConfiguration(transfer_config_))
//        {
//            ROS_ERROR("Could not set manipulator transfer state!");
//            safeShutdown();
//            return false;
//        }
//        task_error_tol_ = 1e-2;
//        activateHQPControl();

//        while(!task_status_changed_)
//            cond_.wait(lock);

//        if(!task_success_)
//        {
//            ROS_ERROR("Could not complete the manipulator transfer state tasks!");
//            safeShutdown();
//            return false;
//        }
//        ROS_INFO("Manipulator transfer state tasks executed successfully.");
//    }

    deactivateHQPControl();
    resetState();
    ROS_INFO("DEMO FINISHED.");
    return true;
}
//--------------------------------------------------------------------------
}//end namespace demo_palletizing


/////////////////////////////////
//           MAIN              //
/////////////////////////////////


//---------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_palletizing");

    demo_palletizing::DemoPalletizing demo;

    ROS_INFO("Demo palletizing node ready");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
//---------------------------------------------------------------------
