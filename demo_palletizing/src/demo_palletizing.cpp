#include <demo_palletizing/demo_palletizing.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <math.h>
#include <limits>
#include <time.h>

#include <hqp_controllers_msgs/TaskGeometry.h>
#include <hqp_controllers_msgs/RemoveTasks.h>
#include <hqp_controllers_msgs/ActivateHQPControl.h>
#include <hqp_controllers_msgs/VisualizeTaskGeometries.h>
#include <hqp_controllers_msgs/LoadTasks.h>
#include <hqp_controllers_msgs/FindCanTask.h>

namespace demo_palletizing
{
//-----------------------------------------------------------------
DemoPalletizing::DemoPalletizing() : task_error_tol_(0.0), task_diff_tol_(1e-5), task_timeout_tol_(0.5)
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
        set_stiffness_clt_ = n_.serviceClient<lbr_fri::SetStiffness>("set_stiffness");
        //get_grasp_interval_clt_.waitForExistence();
        //  velvet_pos_clt_.waitForExistence();
        //velvet_grasp_clt_.waitForExistence();
        set_stiffness_clt_.waitForExistence();
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

    grasp_.obj_frame_ = "world"; //object frame
    grasp_.e_frame_ = "velvet_fingers_palm"; //endeffector frame
    grasp_.e_.setZero(); //endeffector point expressed in the endeffector frame

#ifdef PILE_GRASPING
    grasp_.a_(0) = 1.0; grasp_.a_(1) = 0.0; grasp_.a_(2) = 0.0;
    grasp_.p_(0) = 0.75; grasp_.p_(1) = -0.8; grasp_.p_(2) = 0.24;
#else
    grasp_.v_(0) = 0.0; grasp_.v_(1) = 0.0; grasp_.v_(2) = 1.0; //cylinder normal
    grasp_.p_(0) = 0.9; grasp_.p_(1) = -0.9; grasp_.p_(2) = 0.14; //reference point on the cylinder axis
    grasp_.r1_ = 0.05; grasp_.r2_ = 0.15; //cylinder radii
    grasp_.n1_ = grasp_.v_; grasp_.n2_ = -grasp_.v_; //plane normals
    grasp_.d1_ = 0.2; grasp_.d2_= -0.35; //plane offsets
#endif

    //placement zone
    place_zone_.place_frame_ = "world";
    place_zone_.e_frame_ = "velvet_fingers_palm";
    place_zone_.e_(0) = 0.16;  place_zone_.e_(1) = 0.0;  place_zone_.e_(2) = 0.0;
    place_zone_.p_(0) = 0.75; place_zone_.p_(1) = -0.2; /*-0.2 0.0 0.2*/ place_zone_.p_(2) = 0.15; //reference point on the cylinder axis
    place_zone_.v_(0) = 0.0; place_zone_.v_(1) = 0.0; place_zone_.v_(2) = 1.0; //cylinder normal
    place_zone_.r_ = 0.02;
    place_zone_.n_ = place_zone_.v_;
    place_zone_.d_ = 0.25;

//     place_locations_.resize(3);
//     place_locations_[0](0) = 0.75; place_locations_[0](1) = -0.2; place_locations_[0](2) = 0.15;
//     place_locations_[1](0) = 0.75; place_locations_[1](1) = 0.0; place_locations_[1](2) = 0.15;
//     place_locations_[2](0) = 0.75; place_locations_[2](1) = 0.2; place_locations_[2](2) = 0.15;

     place_locations_.resize(1);
    place_locations_[0](0) = 0.75; place_locations_[0](1) = -0.2; place_locations_[0](2) = 0.15;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setCartesianStiffness(double sx, double sy, double sz, double sa, double sb, double sc)
{
    if(!with_gazebo_)
    {
        lbr_fri::SetStiffness cart_stiffness;

        cart_stiffness.request.sx = sx;
        cart_stiffness.request.sy = sy;
        cart_stiffness.request.sz = sz;
        cart_stiffness.request.sa = sa;
        cart_stiffness.request.sb = sb;
        cart_stiffness.request.sc = sc;
        if(!set_stiffness_clt_.call(cart_stiffness))
        {
            ROS_ERROR("Could not set the cartesian stiffness!");
            return false;
        }
    }

    return true;
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
    get_grasp_interval_clt_.waitForExistence();
    hqp_controllers_msgs::FindCanTask grasp;
    get_grasp_interval_clt_.call(grasp);

    if(!grasp.response.success)
        return false;

#ifdef PILE_GRASPING
    ROS_ASSERT(grasp.response.CanTask.size()==2);
    std::vector<double> data;
    grasp_.obj_frame_ = grasp.response.reference_frame;

    ROS_ASSERT(grasp.response.CanTask[0].g_type == hqp_controllers_msgs::TaskGeometry::POINT);
    data = grasp.response.CanTask[0].g_data;
    grasp_.p_(0) = data[0]; grasp_.p_(1) = data[1]; grasp_.p_(2) = data[2];

    ROS_ASSERT(grasp.response.CanTask[1].g_type == hqp_controllers_msgs::TaskGeometry::LINE);
    data = grasp.response.CanTask[1].g_data;
    grasp_.a_(0) = data[3]; grasp_.a_(1) = data[4]; grasp_.a_(2) = data[5];
#else
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

    //Plane normals need to point in opposit directions to give a closed interval
    ROS_ASSERT((grasp_.n1_.transpose() * grasp_.n2_)  <= 0.0);

#endif
    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::getPileGraspInterval()
{
    //    //get the grasp intervall
    //    hqp_controllers_msgs::FindCanTask grasp;
    //    if(!get_grasp_interval_clt_.call(grasp))
    //    {
    //        ROS_ERROR("Could not obtain the grasp intervall!");
    //        safeShutdown();
    //        return false;
    //    }

    //    ROS_ASSERT(grasp.response.CanTask.size()==4);
    //    std::vector<double> data;
    //    grasp_.obj_frame_ = grasp.response.reference_frame;

    //    //BOTTOM PLANE
    //    ROS_ASSERT(grasp.response.CanTask[0].g_type == hqp_controllers_msgs::TaskGeometry::PLANE);
    //    data = grasp.response.CanTask[0].g_data;
    //    ROS_ASSERT(data.size() == 4);
    //    grasp_.n1_(0) = data[0];  grasp_.n1_(1) = data[1]; grasp_.n1_(2) = data[2];
    //    grasp_.d1_ = data[3];

    //    //TOP PLANE
    //    ROS_ASSERT(grasp.response.CanTask[1].g_type == hqp_controllers_msgs::TaskGeometry::PLANE);
    //    data = grasp.response.CanTask[1].g_data;
    //    ROS_ASSERT(data.size() == 4);
    //    grasp_.n2_(0) = data[0];  grasp_.n2_(1) = data[1]; grasp_.n2_(2) = data[2];
    //    grasp_.d2_ = data[3];

    //    //INNER GRASP CYLINDER
    //    ROS_ASSERT(grasp.response.CanTask[2].g_type == hqp_controllers_msgs::TaskGeometry::CYLINDER);
    //    data = grasp.response.CanTask[2].g_data;
    //    ROS_ASSERT(data.size() == 7);
    //    grasp_.p_(0) = data[0];  grasp_.p_(1) = data[1]; grasp_.p_(2) = data[2];
    //    grasp_.v_(0) = data[3];  grasp_.v_(1) = data[4]; grasp_.v_(2) = data[5];
    //    grasp_.r1_ = data[6];

    //    //OUTER GRASP CYLINDER
    //    ROS_ASSERT(grasp.response.CanTask[3].g_type == hqp_controllers_msgs::TaskGeometry::CYLINDER);
    //    data = grasp.response.CanTask[3].g_data;
    //    ROS_ASSERT(data.size() == 7);
    //    grasp_.r2_ = data[6];

    //    //Plane normals need to point in opposit directions to give a closed interval
    //    ROS_ASSERT((grasp_.n1_.transpose() * grasp_.n2_)  <= 0.0);

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

    //clean up the previous task progress vector;
    t_prog_prev_.resize(0);

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

    //LINK 4 ABOVE HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "link_4_above_horizontal_plane (transfer)";
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(0.7);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = "lbr_iiwa_link_4";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //EE ON HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_horizontal_plane (transfer)";
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 1.5);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(EXTRACT_HEIGHT);
    t_link.link_frame = "world";
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

    //PLACEMENT_CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_in_placement_cylinder";
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.02;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

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
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER APPROACH AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.name = "gripper_approach_axis_alignment";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back( DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(-0.707); t_geom.g_data.push_back(0.707); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE * 10);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE * 0);
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

    t_link.geometries.resize(1);
     t_geom.g_data.resize(1);
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.t_type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
    task.priority = 3;
    task.name = "joint_setpoints (object transfer)";
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1.0;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;

    t_geom.g_data[0] = 0; //2.15
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_1";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0;//0.59
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_2";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0;//-0.97
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_3";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0;//-1.73
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_4";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0;//0.55
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_5";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0.6;//0.62
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_6";
    task.t_links.push_back(t_link);

    t_geom.g_data[0] = 0;//-2.10
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "lbr_iiwa_link_7";
    task.t_links.push_back(t_link);

#ifdef HQP_GRIPPER_JOINT
    t_geom.g_data[0] = 0;
    t_link.geometries[0] = t_geom;
    t_link.link_frame = "velvet_fingers_right";
    task.t_links.push_back(t_link);
#endif

    tasks_.request.tasks.push_back(task);

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks except of the last one
    for(unsigned int i=0; i<tasks_.response.ids.size() - 1;i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize all tasks except of the last one
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size() - 1;i++)
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
    hqp_controllers_msgs::TaskGeometry t_geom;

    //LINK 5 ABOVE HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "lbr_iiwa_link_5_above_horizontal_plane (joint configuration)";
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.02;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 3);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(EXTRACT_HEIGHT);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0.075);
    t_link.link_frame = "lbr_iiwa_link_5";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //LINK 6 ABOVE HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "lbr_iiwa_link_6_above_horizontal_plane (joint configuration)";
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.02;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 3);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(EXTRACT_HEIGHT);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0.1);
    t_link.link_frame = "lbr_iiwa_link_6";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //PALM ABOVE HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "velvet_fingers_palm_above_horizontal_plane (joint configuration)";
    task.is_equality_task = false;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 0.02;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 3);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(EXTRACT_HEIGHT);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::SPHERE;
    t_geom.g_data.push_back(0.04); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0.025);
    t_geom.g_data.push_back(0.11);
    t_link.link_frame = "velvet_fingers_palm";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    t_link.geometries.resize(1);
    t_geom.g_data.resize(1);
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.t_type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
    task.priority = 3;
    task.name = "joint_setpoints";
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 4);
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

    //monitor only the last task
    monitored_tasks_.push_back(tasks_.response.ids.back());

    //visualize all tasks except of the last one
    std::vector<unsigned int> ids = pers_task_vis_ids_;
    for(unsigned int i=0; i<tasks_.response.ids.size() - 1;i++)
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

    //EE ON HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_horizontal_plane (place)";
    task.is_equality_task = true;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(place_zone_.n_(0)); t_geom.g_data.push_back(place_zone_.n_(1)); t_geom.g_data.push_back(place_zone_.n_(2));
    t_geom.g_data.push_back(place_zone_.d_);
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

    //PLACEMENT_CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_in_placement_cylinder (place)";
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 10);

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
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER APPROACH AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.name = "gripper_approach_axis_alignment";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back( DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(-0.707); t_geom.g_data.push_back(0.707); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE * 10);
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 3);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(0.0);
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
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;


    //EE ON ATTACK POINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_attack_point";
    task.is_equality_task = true;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::POINT;
    t_geom.g_data.push_back(grasp_.p_(0) - grasp_.a_(0)*0.2); t_geom.g_data.push_back(grasp_.p_(1) - grasp_.a_(1)*0.2); t_geom.g_data.push_back(grasp_.p_(2) + 0.01);
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

    //GRIPPER APPROACH AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.name = "gripper_approach_axis_alignment";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    //project the approach vector on the x/y plane
    Eigen::Vector3d a;
    a.setZero();
    a(0) = grasp_.a_(0); a(1) = grasp_.a_(1);
    a.normalize();
    t_geom.g_data.push_back(a(0)); t_geom.g_data.push_back(a(1)); t_geom.g_data.push_back(a(2));
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE / 2);
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
//---------------------------------------------------------------------
bool DemoPalletizing::setGripperExtract()
{
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

    //EE ON HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_horizontal_plane (extract)";
    task.is_equality_task = true;
    task.task_frame = "world";
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(EXTRACT_HEIGHT);
    t_link.link_frame = "world";
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

    //PLACEMENT_CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_in_placement_cylinder (place)";
    task.is_equality_task = false;
    task.task_frame = place_zone_.place_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 2);

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
    t_link.link_frame = "world";
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE * 4);
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
bool DemoPalletizing::setGraspApproach()
{
    hqp_controllers_msgs::Task task;
    hqp_controllers_msgs::TaskLink t_link;
    hqp_controllers_msgs::TaskGeometry t_geom;

#ifdef PILE_GRASPING
    ROS_ASSERT(grasp_.p_(0) >= 0.0);
    ROS_ASSERT(grasp_.p_(2) >= 0.23);
    ROS_ASSERT(grasp_.a_(0) >= 0.0);

    //EE ON HORIZONTAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_on_horizontal_plane";
    task.is_equality_task = true;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 2);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_geom.g_data.push_back(0.0); t_geom.g_data.push_back(0.0); t_geom.g_data.push_back(1.0);
    t_geom.g_data.push_back(grasp_.p_(2));
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

    //CONSTRAINT CYLINDER
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.name = "ee_in_constraint_cylinder";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN * 3/2);

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
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(0.14);
    t_geom.g_data.push_back(0.0); t_geom.g_data.push_back(0.0); t_geom.g_data.push_back(1.0);
    t_geom.g_data.push_back(0.005);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER APPROACH AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.name = "gripper_approach_axis_alignment";
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    //project the approach vector on the x/y plane
    Eigen::Vector3d a;
    a.setZero();
    a(0) = grasp_.a_(0); a(1) = grasp_.a_(1);
    a.normalize();
    t_geom.g_data.push_back(a(0)); t_geom.g_data.push_back(a(1)); t_geom.g_data.push_back(a(2));
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //GRIPPER VERTICAL AXIS ALIGNMENT
    task.t_links.clear();
    task.dynamics.d_data.clear();
    task.name = "gripper_vertical_axis_alignment";
    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 1;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
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

#else
    ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);
    ROS_ASSERT(grasp_.n1_.transpose() * grasp_.n2_ < 0.0);

    //LOWER GRASP INTERVAL PLANE
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PROJECTION;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.02;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(DYNAMICS_GAIN / 5);

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
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

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
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

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
    task.dynamics.d_data.push_back(DYNAMICS_GAIN);

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

    task.t_type = hqp_controllers_msgs::Task::COPLANAR;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(2 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::LINE;
    t_geom.g_data.push_back(grasp_.p_(0)); t_geom.g_data.push_back(grasp_.p_(1)); t_geom.g_data.push_back(grasp_.p_(2));
    t_geom.g_data.push_back(grasp_.v_(0)); t_geom.g_data.push_back(grasp_.v_(1)); t_geom.g_data.push_back(grasp_.v_(2));
    t_link.link_frame = grasp_.obj_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(1); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
    t_link.link_frame = grasp_.e_frame_;
    t_link.geometries.push_back(t_geom);
    task.t_links.push_back(t_link);

    tasks_.request.tasks.push_back(task);

    //CONE CONSTRAINT
    task.t_links.clear();
    task.dynamics.d_data.clear();

    task.t_type = hqp_controllers_msgs::Task::PARALLEL;
    task.priority = 2;
    task.is_equality_task = false;
    task.task_frame = grasp_.obj_frame_;
    task.ds = 0.0;
    task.di = 0.05;
    task.dynamics.d_type = hqp_controllers_msgs::TaskDynamics::LINEAR_DYNAMICS;
    task.dynamics.d_data.push_back(4 * DYNAMICS_GAIN);

    t_link.geometries.clear();
    t_geom.g_data.clear();
    t_geom.g_type = hqp_controllers_msgs::TaskGeometry::CONE;
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(0);
    t_geom.g_data.push_back(0); t_geom.g_data.push_back(0); t_geom.g_data.push_back(1);
    t_geom.g_data.push_back(ALIGNMENT_ANGLE);
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
#endif

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
void DemoPalletizing::stateCallback( const hqp_controllers_msgs::TaskStatusArrayPtr& msg)
{
    boost::mutex::scoped_lock lock(manipulator_tasks_m_, boost::try_to_lock);
    if(!lock) return;

    static struct timeval t_stag;
    struct timeval t;
    gettimeofday(&t,0);
    static bool stagnation_flag = false;

    if(!stagnation_flag)
        t_stag = t;

    stagnation_flag = false;
    // double curr = t.tv_sec + 0.000001*t.tv_usec;
    // double stag = t_stag.tv_sec + 0.000001*t_stag.tv_usec;

    // std::cerr<<"t: "<<curr<<std::endl;
    // std::cerr<<"t_stag: "<<stag<<std::endl;

    // std::cerr<<"monitored tasks: ";
    // for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    //   std::cerr<<monitored_tasks_[i]<<" ";

    // std::cerr<<std::endl;
    // std::cerr<<"received tasks: ";
    // std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it2;
    // for(status_it2 = msg->statuses.begin(); status_it2!=msg->statuses.end(); ++status_it2)
    // 	std::cerr<<status_it2->id<<" "<<status_it2->name<<" "<<std::endl;

    // std::cerr<<std::endl;

    Eigen::VectorXd t_prog(monitored_tasks_.size());

    //form the maximum norm over all errors

    for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    {
        //try to find the monitored task id in the given task status message
        std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it;
        for(status_it = msg->statuses.begin(); status_it!=msg->statuses.end(); ++status_it)
            if(monitored_tasks_[i] == status_it->id)
            {
                t_prog(i)=status_it->progress;
                break;
            }

        if(status_it==msg->statuses.end())
        {
            ROS_WARN("No status feedback for monitored task id %d!", monitored_tasks_[i]);
            return; //just so we don't give a false positive task success
        }

    }

    double e = 0.0;
    double e_diff = INFINITY;


    if(monitored_tasks_.size() > 0)
    {
        //task error
        e = t_prog.cwiseAbs().maxCoeff();

        //find the task progress difference between iterations
        if(t_prog_prev_.size() > 0)
            e_diff = (t_prog - t_prog_prev_).cwiseAbs().maxCoeff();

        //std::cerr<<"t_prog: "<<t_prog.transpose()<<"e: "<<e<<std::endl;
        //std::cerr<<"t_prog_prev_: "<<t_prog_prev_.transpose()<<"e_diff: "<<e_diff<<std::endl;
        t_prog_prev_ = t_prog;
    }
    //  std::cout<<" t - t_stag: "<<t.tv_sec - t_stag.tv_sec + 0.000001 * (t.tv_usec - t_stag.tv_usec)<<" task_timeout_tol_: "<<task_timeout_tol_<<std::endl;
    //std::cout<<"e_diff: "<<e_diff<<" e_diff_tol_: "<<task_diff_tol_<<std::endl;

    if(e <= task_error_tol_)
    {
        std::cerr<<std::endl<<"STATE CHANGE:"<<std::endl<<"monitored tasks: ";
        for(unsigned int i=0; i<monitored_tasks_.size(); i++)
            std::cerr<<monitored_tasks_[i]<<" ";

        std::cerr<<std::endl<<"task statuses: "<<std::endl;
        for( std::vector<hqp_controllers_msgs::TaskStatus>::iterator it = msg->statuses.begin(); it!=msg->statuses.end(); ++it)
            std::cerr<<"id: "<<it->id<<" name: "<<it->name<<" progress: "<<it->progress<<std::endl;

        std::cerr<<"e: "<<e<<std::endl<<std::endl;

        // ROS_INFO("Task status switch!");
        task_status_changed_ = true;
        task_success_ = true;
        cond_.notify_one();
    }
    else if(e_diff <= task_diff_tol_) //(task progresses ain't changing no more)
    {
        stagnation_flag = true;
        std::cerr<<"task progress stagnating since:"<<t.tv_sec - t_stag.tv_sec + 0.000001 * (t.tv_usec - t_stag.tv_usec)<<" s, e_diff is: "<<e_diff<<std::endl;
        if((t.tv_sec - t_stag.tv_sec + 0.000001 * (t.tv_usec - t_stag.tv_usec)) > task_timeout_tol_ )
        {
            task_status_changed_ = true;
            task_success_ = true;
            ROS_WARN("Task execution timeout!");
            for(unsigned int i=0; i<monitored_tasks_.size(); i++)
                std::cerr<<monitored_tasks_[i]<<" ";

            std::cerr<<std::endl<<"task statuses: "<<std::endl;
            for( std::vector<hqp_controllers_msgs::TaskStatus>::iterator it = msg->statuses.begin(); it!=msg->statuses.end(); ++it)
                std::cerr<<"id: "<<it->id<<" name: "<<it->name<<" progress: "<<it->progress<<std::endl;

            std::cerr<<"e: "<<e<<std::endl<<std::endl;
            //std::cerr<<"t: "<<"t - t_stag: "<<t.tv_sec - t_stag.tv_sec + 0.000001 * (t.tv_usec - t_stag.tv_usec)<<" task_timeout_tol_: "<<task_timeout_tol_<<std::endl;
            //std::cerr<<"e_diff: "<<e_diff<<" e_diff_tol_: "<<task_diff_tol_<<std::endl<<std::endl;

            stagnation_flag = false;
            cond_.notify_one();
        }
    }

}
//-----------------------------------------------------------------
bool DemoPalletizing::loadPersistentTasks()
{
    hqp_controllers_msgs::LoadTasks persistent_tasks;
    persistent_tasks.request.task_definitions = "task_definitions";
    if(!load_tasks_clt_.call(persistent_tasks))
        return false;

    //visualize (some of) the loaded tasks
    for(unsigned int i=0; i<6;i++)
        pers_task_vis_ids_.push_back(persistent_tasks.response.ids[i+7]);

    if(!visualizeStateTasks(pers_task_vis_ids_))
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::startDemo(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{

    deactivateHQPControl();
    resetState();
    std_srvs::Empty srv;
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

    if(!loadPersistentTasks())
    {
        ROS_ERROR("Could not load persistent tasks!");
        safeShutdown();
        return false;
    }

    if(!with_gazebo_)
    {
        //VELVET INITIAL POSE
        velvet_interface_node::VelvetToPos poscall;
        poscall.request.angle = 0.3;

        if(!velvet_pos_clt_.call(poscall))
        {
            ROS_ERROR("could not call velvet to pos");
            ROS_BREAK();
        }
    }

    for(unsigned int i=0; i<place_locations_.size(); i++)
    {
        place_zone_.p_ = place_locations_[i];

        bool grasp_success = false;
        while(!grasp_success)
        {
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
                if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
                {
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
#if 0
                if(!with_gazebo_)
                    if(!getGraspInterval())
                    {
                        ROS_ERROR("Could not obtain the grasp intervall!");
                        safeShutdown();
                    }
#endif 

                if(!setCartesianStiffness(1000, 1000, 100, 100, 100, 100))
                {
                    safeShutdown();
                    return false;
                }
                if(!setGraspApproach())
                {
                    ROS_ERROR("Could not set the grasp approach!");
                    safeShutdown();
                    return false;
                }
                task_error_tol_ =  1e-3;
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
                //SET GRASP STIFFNESS
                if(!setCartesianStiffness(1000, 50, 15, 100, 100, 10))
                {
                    safeShutdown();
                    return false;
                }
                grasp_success = true; //REEEEEEEEEEMOOOOOOOOOOOOOOOVEEEEEEEEEE!!!!!!!!!
#if 0
                deactivateHQPControl();
                //VELVET GRASP_
                velvet_interface_node::SmartGrasp graspcall;
                graspcall.request.current_threshold_contact = 10;
                graspcall.request.current_threshold_final = 35;
                graspcall.request.max_belt_travel_mm = 90;
                graspcall.request.phalange_delta_rad = 0.02;
                graspcall.request.gripper_closed_thresh = 1.5;
                graspcall.request.check_phalanges = true;

                if(!velvet_grasp_clt_.call(graspcall)) {
                    ROS_ERROR("could not call grasping");
                    ROS_BREAK();
                }
                if(!graspcall.response.success)
                    ROS_ERROR("Grasp failed!");
                else
                {
                    grasp_success = true;
                    ROS_INFO("Grasp aquired.");
                }
#endif
            }
            else
                grasp_success = true;
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
            if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
            {
                safeShutdown();
                return false;
            }
            if(!setObjectExtract())
            {
                ROS_ERROR("Could not set the object extract!");
                safeShutdown();
                return false;
            }

            task_error_tol_ = 2 * 1e-3;
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
            if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
            {
                safeShutdown();
                return false;
            }

            if(!setObjectTransfer())
            {
                ROS_ERROR("Could not set the object transfer!");
                safeShutdown();
                return false;
            }
            task_error_tol_ = 10 * 1e-4;
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
#if 0
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
            if(!setCartesianStiffness(100, 1000, 1000, 100, 100, 100))
            {
                safeShutdown();
                return false;
            }

            if(!setObjectPlace())
            {
                ROS_ERROR("Could not set the object place!");
                safeShutdown();
                return false;
            }
            task_error_tol_ = 5 * 1e-4;
            task_diff_tol_ = 1e-5;
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
            poscall2.request.angle = 0.2;

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
            if(!setCartesianStiffness(1000, 100, 100, 100, 100, 100))
            {
                safeShutdown();
                return false;
            }

            if(!setGripperExtract())
            {
                ROS_ERROR("Could not set the gripper extract!");
                safeShutdown();
                return false;
            }
            task_error_tol_ = 5* 1e-3;
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
#endif
    }
#if 0
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
        if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
        {
            safeShutdown();
            return false;
        }

        if(!setJointConfiguration(transfer_config_))
        {
            ROS_ERROR("Could not set manipulator transfer configuration!");
            safeShutdown();
            return false;
        }
        task_error_tol_ = 1e-2;
        activateHQPControl();

        while(!task_status_changed_)
            cond_.wait(lock);

        if(!task_success_)
        {
            ROS_ERROR("Could not complete the manipulator transfer configuration tasks!");
            safeShutdown();
            return false;
        }
        ROS_INFO("Manipulator transfer configuration tasks executed successfully.");
    }
#endif
    deactivateHQPControl();
    resetState();
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
