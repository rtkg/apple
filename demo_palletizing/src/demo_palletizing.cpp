#include <demo_palletizing/demo_palletizing.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <math.h>
#include <limits>

#include <hqp_controllers_msgs/TaskGeometry.h>
#include <hqp_controllers_msgs/RemoveTasks.h>
#include <hqp_controllers_msgs/ActivateHQPControl.h>
#include <hqp_controllers_msgs/VisualizeTaskGeometries.h>
#include <hqp_controllers_msgs/LoadTasks.h>
#include <hqp_controllers_msgs/FindCanTask.h>

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
    nh_.param<bool>("with_gazebo", with_gazebo_,false);
    if(with_gazebo_)
        ROS_INFO("Demo palletizing running in Gazebo.");

    //initialize variables
    task_status_changed_ = false;
    task_success_ = false;

    //register general callbacks
    start_demo_srv_ = nh_.advertiseService("start_demo", &DemoPalletizing::startDemo, this);;
    task_status_sub_ = n_.subscribe("task_status_array", 1, &DemoPalletizing::stateCallback, this);
    set_tasks_clt_ = n_.serviceClient<hqp_controllers_msgs::SetTasks>("set_tasks");
    remove_tasks_clt_ = n_.serviceClient<hqp_controllers_msgs::RemoveTasks>("remove_tasks");
    activate_hqp_control_clt_ = n_.serviceClient<hqp_controllers_msgs::ActivateHQPControl>("activate_hqp_control");
    visualize_task_geometries_clt_ = n_.serviceClient<hqp_controllers_msgs::VisualizeTaskGeometries>("visualize_task_geometries");
    set_gazebo_physics_clt_ = n_.serviceClient<gazebo_msgs::SetPhysicsProperties>("set_physics_properties");
    load_tasks_clt_ = n_.serviceClient<hqp_controllers_msgs::LoadTasks>("load_tasks");
    reset_hqp_control_clt_ = n_.serviceClient<std_srvs::Empty>("reset_hqp_control");

    if(!with_gazebo_)
    {
        get_grasp_interval_clt_ = n_.serviceClient<hqp_controllers_msgs::FindCanTask>("get_grasp_interval");
        velvet_pos_clt_ = n_.serviceClient<velvet_interface_node::VelvetToPos>("velvet_pos");
        velvet_grasp_clt_ = n_.serviceClient<velvet_interface_node::SmartGrasp>("velvet_grasp");

        get_grasp_interval_clt_.waitForExistence();
        velvet_pos_clt_.waitForExistence();
        velvet_grasp_clt_.waitForExistence();
    }
    else
    {
        //if gazebo is used, set the simulated gravity to zero in order to prevent gazebo's joint drifting glitch
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

    set_tasks_clt_.waitForExistence();
    remove_tasks_clt_.waitForExistence();
    activate_hqp_control_clt_.waitForExistence();
    visualize_task_geometries_clt_.waitForExistence();
    load_tasks_clt_.waitForExistence();
    reset_hqp_control_clt_.waitForExistence();

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
    /*
    sensing_config_[0] = 1.27;
    sensing_config_[1] = -1.9;
    sensing_config_[2] = -0.52;
    sensing_config_[3] = -1.9;
    sensing_config_[4] = -0.46;
    sensing_config_[5] = 0.79;
    sensing_config_[6] = -1.48;
    */
    sensing_config_[0] = 1.274;
    sensing_config_[1] = -1.85;
    sensing_config_[2] = -0.367;
    sensing_config_[3] = -1.885;
    sensing_config_[4] = -0.122;
    sensing_config_[5] = 0.91;
    sensing_config_[6] = -1.483;
#ifdef HQP_GRIPPER_JOINT
    sensing_config_[7] = 0.1;
#endif

    //Grasp intervall specification
    // grasp_.obj_frame_ = "world";  //object frame
    // grasp_.e_frame_ = "velvet_fingers_palm"; //endeffector frame
    // grasp_.e_.setZero();                     //endeffector point expressed in the endeffector frame
    // grasp_.v_(0) = 0.0; grasp_.v_(1) = 0.0; grasp_.v_(2) = 0.0; //cylinder normal
    // grasp_.p_(0) = 0.0; grasp_.p_(1) = -0.0; grasp_.p_(2) = 0.00; //reference point on the cylinder axis
    // grasp_.r1_ = 0.0; grasp_.r2_ = 0.0;              //cylinder radii
    // grasp_.n1_ = grasp_.v_; grasp_.n2_ = grasp_.v_;  //plane normals
    // grasp_.d1_ = 0.0; grasp_.d2_= 0.0;
    
    //plane offsets

    grasp_.obj_frame_ = "world"; //object frame
    grasp_.e_frame_ = "velvet_fingers_palm"; //endeffector frame
    grasp_.e_.setZero(); //endeffector point expressed in the endeffector frame
    grasp_.v_(0) = 0.0; grasp_.v_(1) = 0.0; grasp_.v_(2) = 1.0; //cylinder normal
    grasp_.p_(0) = 1.0; grasp_.p_(1) = -0.9; grasp_.p_(2) = 0.14; //reference point on the cylinder axis
    grasp_.r1_ = 0.05; grasp_.r2_ = 0.15; //cylinder radii
    grasp_.n1_ = grasp_.v_; grasp_.n2_ = -grasp_.v_; //plane normals
    grasp_.d1_ = 0.2; grasp_.d2_= -0.35; //plane offsets

   //placement zone
    place_zone_.place_frame_ = "world";
    place_zone_.e_frame_ = "velvet_fingers_palm";
    place_zone_.e_.setZero();
    place_zone_.p_(0) = 0.8; place_zone_.p_(1) = 0.0; place_zone_.p_(2) = 0.13; //reference point on the cylinder axis
    place_zone_.v_(0) = 0.0; place_zone_.v_(1) = 0.0; place_zone_.v_(2) = 1.0; //cylinder normal
    place_zone_.r_ = 0.05;
    place_zone_.n_ = place_zone_.v_;
    place_zone_.d_ = 0.25;
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
    resetState();
    std_srvs::Empty srv;
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();
    ROS_BREAK(); //I must break you ... ros::shutdown() doesn't seem to do the job
}
//-----------------------------------------------------------------
bool DemoPalletizing::getGraspInterval()
{
    //get the grasp intervall
    hqp_controllers_msgs::FindCanTask grasp;
    if(!get_grasp_interval_clt_.call(grasp))
    {
        ROS_ERROR("Could not obtain the grasp intervall!");
        safeShutdown();
        return false;
    }

    ROS_ASSERT(grasp.response.CanTask.size()==4);
    std::vector<double> data;
    grasp_.obj_frame_ = grasp.response.reference_frame;

    //BOTTOM PLANE
    ROS_ASSERT(grasp.response.CanTask[0].g_type == hqp_controllers_msgs::TaskGeometry::PLANE);
    data = grasp.response.CanTask[0].g_data;
    ROS_ASSERT(data.size() == 4);
    grasp_.n1_(0) = data[0];  grasp_.n1_(1) = data[1]; grasp_.n1_(2) = data[2];
    grasp_.d1_ = data[3];

    //TOP PLANE
    ROS_ASSERT(grasp.response.CanTask[1].g_type == hqp_controllers_msgs::TaskGeometry::PLANE);
    data = grasp.response.CanTask[1].g_data;
    ROS_ASSERT(data.size() == 4);
    grasp_.n2_(0) = data[0];  grasp_.n2_(1) = data[1]; grasp_.n2_(2) = data[2];
    grasp_.d2_ = data[3];

    //INNER GRASP CYLINDER
    ROS_ASSERT(grasp.response.CanTask[2].g_type == hqp_controllers_msgs::TaskGeometry::CYLINDER);
    data = grasp.response.CanTask[2].g_data;
    ROS_ASSERT(data.size() == 7);
    grasp_.p_(0) = data[0];  grasp_.p_(1) = data[1]; grasp_.p_(2) = data[2];
    grasp_.v_(0) = data[3];  grasp_.v_(1) = data[4]; grasp_.v_(2) = data[5];
    grasp_.r1_ = data[6];

    //OUTER GRASP CYLINDER
    ROS_ASSERT(grasp.response.CanTask[3].g_type == hqp_controllers_msgs::TaskGeometry::CYLINDER);
    data = grasp.response.CanTask[3].g_data;
    ROS_ASSERT(data.size() == 7);
    grasp_.r2_ = data[6];

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::resetState()
{
    hqp_controllers_msgs::RemoveTasks rem_t_srv;
    for(unsigned int i=0; i<tasks_.response.ids.size(); i++)
        rem_t_srv.request.ids.push_back(tasks_.response.ids[i]);

    remove_tasks_clt_.call(rem_t_srv);
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
bool DemoPalletizing::visualizeStateTasks(std::vector<unsigned int> const& ids)
{
    //send a visualization message to show the tasks in Rviz
    hqp_controllers_msgs::VisualizeTaskGeometries vis_srv;
    for(unsigned int i=0; i<ids.size();i++)
        vis_srv.request.ids.push_back(ids[i]);

    visualize_task_geometries_clt_.call(vis_srv);
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
bool DemoPalletizing::setObjectTransfer()
{
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

    //GRASP TRANSFER PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(OBJECT_TRANSFER_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(0.5);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    //second plane is just to avoid drifting of the controller
    t_geom.g_data.clear();
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(-1);
    t_geom.g_data.push_back(-0.52);
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(OBJECT_TRANSFER_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(place_zone_.e_(0)); t_geom.g_data.push_back(place_zone_.e_(1)); t_geom.g_data.push_back(place_zone_.e_(2));
    t_link.link_frame = place_zone_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(place_zone_.p_(0)); t_geom.g_data.push_back(place_zone_.p_(1)); t_geom.g_data.push_back(place_zone_.p_(2));
    t_geom.g_data.push_back(place_zone_.v_(0)); t_geom.g_data.push_back(place_zone_.v_(1)); t_geom.g_data.push_back(place_zone_.v_(2));
    t_geom.g_data.push_back(place_zone_.r_);
    t_link.link_frame = place_zone_.place_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::ORIENTATION;
    task.priority = 2;
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * OBJECT_TRANSFER_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(GRIPPER_ALIGNMENT_ANGLE);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize all tasks
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        ids.push_back(tasks_.response.ids[i]);

    if(!visualizeStateTasks(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setObjectPlace()
{
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

    //GRASP PLACE PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(OBJECT_PLACE_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(-1);
    t_geom.g_data.push_back(-0.25);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(OBJECT_TRANSFER_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(place_zone_.e_(0)); t_geom.g_data.push_back(place_zone_.e_(1)); t_geom.g_data.push_back(place_zone_.e_(2));
    t_link.link_frame = place_zone_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(place_zone_.p_(0)); t_geom.g_data.push_back(place_zone_.p_(1)); t_geom.g_data.push_back(place_zone_.p_(2));
    t_geom.g_data.push_back(place_zone_.v_(0)); t_geom.g_data.push_back(place_zone_.v_(1)); t_geom.g_data.push_back(place_zone_.v_(2));
    t_geom.g_data.push_back(place_zone_.r_);
    t_link.link_frame = place_zone_.place_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::ORIENTATION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * OBJECT_PLACE_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(GRIPPER_ALIGNMENT_ANGLE);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize all tasks
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        ids.push_back(tasks_.response.ids[i]);

    if(!visualizeStateTasks(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setObjectExtract()
{
    ROS_ERROR("Error in DemoPalletizing::setObjectExtract(): not implemented yet!");
    ROS_BREAK();

    return true;
}
//---------------------------------------------------------------------
bool DemoPalletizing::setGripperExtract()
{
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

    //GRIPPER EXTRACT PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(GRIPPER_EXTRACT_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(0.6);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    // CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(OBJECT_TRANSFER_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(place_zone_.p_(0)); t_geom.g_data.push_back(place_zone_.p_(1)); t_geom.g_data.push_back(place_zone_.p_(2));
    t_geom.g_data.push_back(place_zone_.v_(0)); t_geom.g_data.push_back(place_zone_.v_(1)); t_geom.g_data.push_back(place_zone_.v_(2));
    t_geom.g_data.push_back(place_zone_.r_);
    t_link.link_frame = place_zone_.place_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(place_zone_.e_(0)); t_geom.g_data.push_back(place_zone_.e_(1)); t_geom.g_data.push_back(place_zone_.e_(2));
    t_link.link_frame = place_zone_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::ORIENTATION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * GRIPPER_EXTRACT_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(GRIPPER_ALIGNMENT_ANGLE);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize all tasks
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        ids.push_back(tasks_.response.ids[i]);

    if(!visualizeStateTasks(ids))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setGraspApproach()
{
    ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);
    ROS_ASSERT(grasp_.n1_.transpose() * grasp_.n2_ < 0.0);

    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

    //LOWER GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n1_(0)); t_geom.g_data.push_back(grasp_.n1_(1)); t_geom.g_data.push_back(grasp_.n1_(2));
    t_geom.g_data.push_back(grasp_.d1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //UPPER GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(grasp_.n2_(0)); t_geom.g_data.push_back(grasp_.n2_(1)); t_geom.g_data.push_back(grasp_.n2_(2));
    t_geom.g_data.push_back(grasp_.d2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //INNER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0)); t_geom.g_data.push_back(grasp_.v_(1)); t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r1_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //OUTER CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.e_(0)); t_geom.g_data.push_back(grasp_.e_(1)); t_geom.g_data.push_back(grasp_.e_(2));
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0)); t_geom.g_data.push_back(grasp_.v_(1)); t_geom.g_data.push_back(grasp_.v_(2));
    t_geom.g_data.push_back(grasp_.r2_);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //COPLANAR LINES CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = true;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0)); t_geom.g_data.push_back(grasp_.v_(1)); t_geom.g_data.push_back(grasp_.v_(2));
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::ORIENTATION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * GRASP_APPROACH_DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(GRIPPER_ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize all tasks
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        ids.push_back(tasks_.response.ids[i]);

    if(!visualizeStateTasks(ids))
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

    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    t_link.geometries.resize(1);
    hqp_controllers_msgs::TaskGeometry t_geom;
    t_geom.g_data.resize(1);

    task.t_type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
    task.priority = 2;
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1.0;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(JOINT_SETPOINT_DYNAMICS_GAIN);
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;

    t_geom.g_data[0] = joints[0];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_1";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[1];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_2";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[2];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_3";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[3];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_4";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[4];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_5";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[5];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_6";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = joints[6];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_7";
    task.t_links.push_back(t_link);

#ifdef HQP_GRIPPER_JOINT
    t_geom.g_data[0] = joints[7];
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "velvet_fingers_right";
    task.t_links.push_back(t_link);
#endif

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor the task
    monitored_tasks_.push_back(tasks_.response.ids[0]);

    //could start joint target visualization here, but its messed up for the joints anyway ...

    return true;
}
//-----------------------------------------------------------------
void DemoPalletizing::stateCallback( const hqp_controllers_msgs::TaskStatusArrayPtr& msg)
{
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);

    //    std::cerr<<"monitor tasks: ";
    //      for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    //          std::cerr<<monitored_tasks_[i]<<" ";

    //      std::cerr<<std::endl;

    //form the maximum norm over all errors
    double e = 0.0;
    for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    {
        //try to find the monitored task id in the given task status message
        std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it;
        for(status_it = msg->statuses.begin(); status_it!=msg->statuses.end(); ++status_it)
            if(monitored_tasks_[i] == status_it->id)
            {
                //std::cerr<<"task id: "<<status_it->id<<" sse: "<<status_it->sse<<std::endl;
                //found the corresponding task in the status message
                if (status_it->progress > e)
                    e = status_it->progress;

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
        std::cerr<<std::endl<<"STATE CHANGE:"<<std::endl<<"monitored tasks: ";
        for(unsigned int i=0; i<monitored_tasks_.size(); i++)
            std::cerr<<monitored_tasks_[i]<<" ";

        std::cerr<<std::endl<<"task statuses: "<<std::endl;
        for( std::vector<hqp_controllers_msgs::TaskStatus>::iterator it = msg->statuses.begin(); it!=msg->statuses.end(); ++it)
            std::cerr<<"id: "<<it->id<<" progress: "<<it->progress<<std::endl;

        std::cerr<<"e: "<<e<<std::endl<<std::endl;

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
bool DemoPalletizing::loadPersistentTasks()
{
    hqp_controllers_msgs::LoadTasks persistent_tasks;
    persistent_tasks.request.task_definitions = "task_definitions";
    if(!load_tasks_clt_.call(persistent_tasks))
        return false;

    //visualize (some of) the loaded tasks
    for(unsigned int i=0; i<5;i++)
        pers_task_vis_ids_.push_back(persistent_tasks.response.ids[i+7]);

    if(!visualizeStateTasks(pers_task_vis_ids_))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::startDemo(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
    deactivateHQPControl();
    if(!loadPersistentTasks())
    {
        ROS_ERROR("Could not load persistent tasks!");
        safeShutdown();
        return false;
    }

#if 0
    {//MANIPULATOR HOME CONFIGURATION
        ROS_INFO("Trying to home the manipulator.");
        boost::mutex::scoped_lock lock(manipulator_tasks_m_);
        task_status_changed_ = false;
        task_success_ = false;
        deactivateHQPControl(); //better safe than sorry ...
        if(!resetState())
        {
            ROS_ERROR("Could not reset the state!");
            safeShutdown();
            return false;
        }
        if(!setJointConfiguration(home_config_))
        {
            ROS_ERROR("Could not set manipulator home state!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the manipulator home state tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Manipulator home state tasks executed successfully.");
    }


    if(!with_gazebo_)
    {
        //VELVET INITIAL POSE
        velvet_interface_node::VelvetToPos poscall;
        poscall.request.angle = 0.4;

        if(!velvet_pos_clt_.call(poscall))
        {
            ROS_ERROR("could not call velvet to pos");
            ROS_BREAK();
        }
    }

    {//MANIPULATOR TRANSFER CONFIGURATION
        ROS_INFO("Trying to put the manipulator in transfer configuration.");
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
        if(!setJointConfiguration(transfer_config_))
        {
            ROS_ERROR("Could not set manipulator transfer state!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the manipulator transfer state tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Manipulator transfer state tasks executed successfully.");
    }
#endif
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
        if(!with_gazebo_)
            getGraspInterval();

        if(!setGraspApproach())
        {
            ROS_ERROR("Could not set the grasp approach!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 5*1e-3;
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

    if(!with_gazebo_)
    {
        //VELVET GRASP
        velvet_interface_node::SmartGrasp graspcall;
        graspcall.request.current_threshold_contact = 15;
        graspcall.request.current_threshold_final = 35;
        graspcall.request.max_belt_travel_mm = 100;
        graspcall.request.phalange_delta_rad = 0.02;
        graspcall.request.gripper_closed_thresh = 1.5;
        graspcall.request.check_phalanges = true;

        if(!velvet_grasp_clt_.call(graspcall)) {
            ROS_ERROR("could not call grasping");
            ROS_BREAK();
        }
    }
    // {//OBJECT EXTRACT
    //   ROS_INFO("Trying object extract.");
    //   boost::mutex::scoped_lock lock(manipulator_tasks_m_);
    //   task_status_changed_ = false;
    //   task_success_ = false;
    //   deactivateHQPControl();
    //   if(!resetState())
    //     {
    // 	  ROS_ERROR("Could not reset the state!");
    // 	  safeShutdown();
    // 	  return false;
    //     }
    //   if(!setObjectExtract())
    //     {
    // 	  ROS_ERROR("Could not set the object extract!");
    // 	  safeShutdown();
    // 	  return false;
    //     }
    //   task_error_tol_ = 1e-2;
    //   activateHQPControl();

    //   while(!task_status_changed_)
    // 	cond_.wait(lock);

    //   if(!task_success_)
    //     {
    // 	  ROS_ERROR("Could not complete the object extract tasks!");
    // 	  safeShutdown();
    // 	  return false;
    //     }
    //   ROS_INFO("Object extract tasks executed successfully.");
    // }

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
        task_error_tol_ = 5 * 1e-3;
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


    {//OBJECT PLACE
        ROS_INFO("Trying object place.");
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
        if(!setObjectPlace())
        {
            ROS_ERROR("Could not set the object place!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 5 * 1e-3;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the object place tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Object place tasks executed successfully.");
    }

    if(!with_gazebo_)
    {
        velvet_interface_node::VelvetToPos poscall2;
        poscall2.request.angle = 0.4;

        if(!velvet_pos_clt_.call(poscall2))
        {
            ROS_ERROR("could not call velvet to pos");
            ROS_BREAK();
        }
    }

    {//GRIPPER EXTRACT
        ROS_INFO("Trying gripper extract.");
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
        if(!setGripperExtract())
        {
            ROS_ERROR("Could not set the gripper extract!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the gripper extract tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Gripper extract tasks executed successfully.");
    }

    {//MANIPULATOR TRANSFER CONFIGURATION
        ROS_INFO("Trying to put the manipulator in transfer configuration.");
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
        if(!setJointConfiguration(transfer_config_))
        {
            ROS_ERROR("Could not set manipulator transfer state!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the manipulator transfer state tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Manipulator transfer state tasks executed successfully.");
    }

    deactivateHQPControl();
    resetState();
    std_srvs::Empty srv;
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

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
