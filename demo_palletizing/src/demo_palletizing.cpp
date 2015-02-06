#include <demo_palletizing/demo_palletizing.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <math.h>
#include <limits>

#include <hqp_controllers_msgs/RemoveTasks.h>
#include <hqp_controllers_msgs/RemoveTaskObjects.h>
#include <hqp_controllers_msgs/ActivateHQPControl.h>
#include <hqp_controllers_msgs/VisualizeTaskObjects.h>

namespace demo_palletizing
{
//-----------------------------------------------------------------
DemoPalletizing::DemoPalletizing()
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

    //initialize stuff
    manipulator_joint_map_["lbr_iiwa_joint_1"] = std::pair<std::string, std::string>("lbr_iiwa_link_0", "lbr_iiwa_link_1");
    manipulator_joint_map_["lbr_iiwa_joint_2"] = std::pair<std::string, std::string>("lbr_iiwa_link_1", "lbr_iiwa_link_2");
    manipulator_joint_map_["lbr_iiwa_joint_3"] = std::pair<std::string, std::string>("lbr_iiwa_link_2", "lbr_iiwa_link_3");
    manipulator_joint_map_["lbr_iiwa_joint_4"] = std::pair<std::string, std::string>("lbr_iiwa_link_3", "lbr_iiwa_link_4");
    manipulator_joint_map_["lbr_iiwa_joint_5"] = std::pair<std::string, std::string>("lbr_iiwa_link_4", "lbr_iiwa_link_5");
    manipulator_joint_map_["lbr_iiwa_joint_6"] = std::pair<std::string, std::string>("lbr_iiwa_link_5", "lbr_iiwa_link_6");
    manipulator_joint_map_["lbr_iiwa_joint_7"] = std::pair<std::string, std::string>("lbr_iiwa_link_6", "lbr_iiwa_link_7");
    manipulator_joint_map_["velvet_fingers_joint_1"] = std::pair<std::string, std::string>("velvet_fingers_palm", "velvet_fingers_right");

    //configs have to be within the safety margins of the joint limits
    home_config_ = std::vector<double>(8, 0.1); //7 arm joint + 1 velvet fingers gripper joint
    transfer_config_ = std::vector<double>(8);
    transfer_config_[0] = 1.58;
    transfer_config_[1] = 1.9;
    transfer_config_[2] = -1.54;
    transfer_config_[3] = 1.9;
    transfer_config_[4] = 0.35;
    transfer_config_[5] = 1.8;
    transfer_config_[6] = 0.0;
    transfer_config_[7] = 0.1;

    sensing_config_ = std::vector<double>(8);
    sensing_config_[0] = 1.8;
    sensing_config_[1] = -1.63;
    sensing_config_[2] = -0.3;
    sensing_config_[3] = -1.33;
    sensing_config_[4] = -0.36;
    sensing_config_[5] = 0.93;
    sensing_config_[6] = -1.60;
    sensing_config_[7] = 0.1;

    //Grasp intervall specification
    grasp_.obj_frame_ = "load_pallet_base";  //object frame
    grasp_.e_frame_ = "velvet_fingers_palm"; //endeffector frame
    grasp_.e_.setZero();                     //endeffector point expressed in the endeffector frame
    grasp_.v_(0) = 0.0; grasp_.v_(1) = 0.0; grasp_.v_(2) = 1.0; //cylinder normal
    grasp_.p_(0) = 0.0; grasp_.p_(1) = 0.0; grasp_.p_(2) = 0.13; //reference point on the cylinder axis
    grasp_.r1_ = 0.2; grasp_.r2_ = 0.3;              //cylinder radii
    grasp_.n1_ = grasp_.v_; grasp_.n2_ = grasp_.v_;  //plane normals
    grasp_.d1_ = 0.25; grasp_.d2_= 0.35;              //plane offsets
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

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::visualizeStateTaskObjects()
{
    //send a visualization message to show the task objects in Rviz
    hqp_controllers_msgs::VisualizeTaskObjects vis_srv;
    for(unsigned int i=0; i<task_objects_.response.ids.size();i++)
        vis_srv.request.ids.push_back(task_objects_.response.ids[i]);

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
bool DemoPalletizing::setGraspApproach()
{
    ROS_ASSERT(grasp_.r1_ <= grasp_.r2_);

    //TASK OBJECTS
    hqp_controllers_msgs::TaskObject t_obj;
    hqp_controllers_msgs::TaskGeometry t_geom;
    //endeffector point
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.e_frame_;
    t_geom.type = hqp_controllers_msgs::TaskGeometry::POINT;
    std::vector<double> data(3);
    data[0] = grasp_.e_(0); data[1] = grasp_.e_(1); data[2] = grasp_.e_(2);
    t_geom.data = data;
    t_obj.geometries.push_back(t_geom);
    task_objects_.request.objs.push_back(t_obj);

    //first cylinder
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::CYLINDER;
    //first 3 entries of data are the cylinder axis's reference point, the second 3 entries the direction, the last one is the radius
    data.resize(7);
    data[0] = grasp_.p_(0); data[1] = grasp_.p_(1); data[2] = grasp_.p_(2);
    data[3] = grasp_.v_(0); data[4] = grasp_.v_(1); data[5] = grasp_.v_(2);
    data[6] = grasp_.r1_;
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //second cylinder is the same apart of a different radius
    data[6] = grasp_.r2_;
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //first plane
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::PLANE;
    //the normal is given as the first 3 entries of data, the offset as the 4th one
    data.resize(4);
    data[0] = grasp_.n1_(0); data[1] = grasp_.n1_(1); data[2] = grasp_.n1_(2);
    data[3] = grasp_.d1_;
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::PLANE;
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //second plane
    data[0] = grasp_.n2_(0); data[1] = grasp_.n2_(1); data[2] = grasp_.n2_(2);
    data[3] = grasp_.d2_;
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //add a cone to constrain the vertical axis of the gripper
    t_obj.root = "world";
    t_obj.link = "world";
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::CONE;
    //first 3 entries of link_data are the cone's reference point, the second 3 entries the direction, the last entry is the opening angle
    data.resize(7);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.0; data[4] = 0.0; data[5] = 1.0; //gripper axis should be vertical
    data[6] = 0.085; //corresponds to an angle of 5 degree
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //add a line describing the vertical axis of the gripper
    t_obj.root = "world";
    t_obj.link = grasp_.e_frame_;
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::LINE;
    //first 3 entries of link_data are the line's reference point, the second 3 entries the direction
    data.resize(6);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 0.0; data[3] = 0.0; data[5] = 1.0;
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //add a line describing an axis to which the gripper axis should be aligned to
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.obj_frame_;
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::LINE;
    data.resize(6);
    data[0] = grasp_.p_(0); data[1] = grasp_.p_(1); data[2] = grasp_.p_(2);
    data[3] = grasp_.v_(0); data[4] = grasp_.v_(1); data[5] = grasp_.v_(2);
    t_obj.geometries[0].data = data;
    task_objects_.request.objs.push_back(t_obj);

    //add a line representing the gripper's approach axis
    t_obj.root = grasp_.obj_frame_;
    t_obj.link = grasp_.e_frame_;
    t_obj.geometries[0].type = hqp_controllers_msgs::TaskGeometry::LINE;
    data.resize(6);
    data[0] = 0.0; data[1] = 0.0; data[2] = 0.0;
    data[3] = 1.0; data[4] = 0.0; data[5] = 0.0;
    t_obj.geometries[0].data = data;
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

    //send the filled task message to the controller
    if(!sendStateTasks())
        return false;

    //monitor all tasks
    for(unsigned int i=0; i<tasks_.response.ids.size();i++)
        monitored_tasks_.push_back(tasks_.response.ids[i]);

    //visualize the task objects
    if(!visualizeStateTaskObjects())
        return false;

    return true;
}
//-----------------------------------------------------------------
bool DemoPalletizing::setJointConfiguration(std::vector<double> const& joints)
{
    ROS_ASSERT(joints.size() == manipulator_joint_map_.size());

    //fill in the task_objects_
    unsigned int jnt_id = 0;
    std::map<std::string, std::pair<std::string, std::string> >::iterator it;
    for(it = manipulator_joint_map_.begin(); it!=manipulator_joint_map_.end(); ++it)
    {
        hqp_controllers_msgs::TaskObject t_obj;
        hqp_controllers_msgs::TaskGeometry t_geom;
        t_obj.root = it->second.first;
        t_obj.link = it->second.second;
        t_geom.type = hqp_controllers_msgs::TaskGeometry::JOINT_POSITION;

        //Joint position is described by a frame expressed in the link with z pointing in the joint axis and zero angle pointing in x
        //link_data.tail<6>() is the initial transformation of the joint to the root
        //the position angle q = data[0]
        t_geom.data = std::vector<double>(13, 0.0); //too lazy to fill in the geometric data properly now ... will mess up the visualization but well ...
        t_geom.data[0] = joints[jnt_id];
        jnt_id++;
        t_obj.geometries.push_back(t_geom);
        task_objects_.request.objs.push_back(t_obj);
    }
    //send the filled task object message to the controller
    if(!sendStateTaskObjects())
        return false;

    //fill in the tasks_
    for(unsigned int i=0; i<joints.size(); i++)
    {
        hqp_controllers_msgs::Task task;
        task.type = hqp_controllers_msgs::Task::JOINT_SETPOINT;
        task.priority = 2;
        task.sign = "=";
        task.t_obj_ids.push_back(task_objects_.response.ids[i]);
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

    //could start visualization here, but its messed up for the joints anyway ...

    return true;
}
//-----------------------------------------------------------------
double DemoPalletizing::maximumNorm(std::vector<double>const& e)
{
    double e_max=0.0;
    for(unsigned int i=0; i<e.size();i++)
        if(fabs(e[i]) > e_max)
            e_max = fabs(e[i]);
}
//-----------------------------------------------------------------
void DemoPalletizing::stateCallback( const hqp_controllers_msgs::TaskStatusesPtr& msg)
{
    boost::mutex::scoped_lock lock(manipulator_tasks_m_);

    ///FIXXXXMEEEE THE TASK ERROR NEEDS TO BE REWORKED


    //form the maximum norm over all errors
    std::vector<double> e;

    //     for(unsigned int i=0; i<monitored_tasks_.size(); i++)
    //     {
    //         //try to find the monitored task id in the given task status message
    //         std::vector<hqp_controllers_msgs::TaskStatus>::const_iterator status_it;
    //         for(status_it = msg->statuses.begin(); status_it!=msg->statuses.end(); ++status_it)
    //         {
    //             if(monitored_tasks_[i] == status_it->id)
    //             {
    //                 //found the corresponding task in the status message, now check the sign to evaluate the error
    //                 if(status_it->sign == "=")
    //                 {
    //                     for(unsigned int j=0; j<status_it->e.size(); j++)
    //                         e.push_back(status_it->e[j]);

    //                 }
    //                 else if(status_it->sign == "<=")
    //                 {
    //                     for(unsigned int j=0; j<status_it->e.size(); j++)
    //                         if(status_it->e[j] < 0.0)
    //                             e.push_back(status_it->e[j]);

    //                 }
    //                 else if(status_it->sign == ">=")
    //                 {
    //                               for(unsigned int j=0; j<status_it->e.size(); j++)
    //                     if(status_it->e[j] > 0.0)
    //                         e.push_back(status_it->e[j]);
    //                 }
    //                 else
    //                     ROS_BREAK();

    //                 break;
    //             }

    //             if(status_it==msg->statuses.end())
    //             {
    //                 ROS_DEBUG("No status feedback for monitored task id %d!", monitored_tasks_[i]);
    //                 e.push_back(TASK_ERROR_TOL * 2); //just so we don't give a false positive task success
    //             }

    //         }

    //     }

    if(maximumNorm(e) < TASK_ERROR_TOL)
    {
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
    ros::Duration(5.0).sleep();

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
    ros::Duration(5.0).sleep();

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
    ros::Duration(5.0).sleep();

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
