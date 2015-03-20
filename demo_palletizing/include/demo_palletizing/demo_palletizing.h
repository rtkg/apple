#ifndef DEMO_PALLETIZING_H
#define DEMO_PALLETIZING_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <vector>
#include <std_srvs/Empty.h>
#include <hqp_controllers_msgs/TaskStatusArray.h>
#include <hqp_controllers_msgs/SetTasks.h>
#include <Eigen/Core>
#include <velvet_interface_node/SmartGrasp.h>
#include <velvet_interface_node/VelvetToPos.h>
#include <lbr_fri/SetStiffness.h>

namespace demo_palletizing
{
//-----------------------------------------------------------
//#define HQP_GRIPPER_JOINT 1

#define PILE_GRASPING 1

#define DYNAMICS_GAIN  -0.5
#define ALIGNMENT_ANGLE  0.05

#define EXTRACT_HEIGHT 0.6
//-----------------------------------------------------------
///**To simplify, a grasp intervall is given as two concentric cylinders, described by axis v and a point p on the axis (referenced in a static obj_frame), and two planes. The controller will try to bring endeffector point e, expressed in frame e_frame, inside the intervall described by the two cylinders and the planes (i.e., inside the shell formed by the cylinders and in between the planes described by n^Tx - d = 0)*/
struct GraspInterval
{
    std::string obj_frame_; //object frame
    std::string e_frame_; //endeffector frame
    Eigen::Vector3d e_; //endeffector point expressed in e_frame_
#ifdef PILE_GRASPING
    Eigen::Vector3d p_; //pile attack point
    Eigen::Vector3d a_; //approach axis
#else
    Eigen::Vector3d v_; //cylinder axis
    Eigen::Vector3d p_; //cylinder reference point
    double r1_, r2_; //cylinder radii r2 !> r1

    Eigen::Vector3d n1_, n2_; //plane normals
    double d1_, d2_; //plane offsets d1 !> d2
#endif
};
//-----------------------------------------------------------
struct PlaceInterval
{
    std::string place_frame_;
    std::string e_frame_; //endeffector frame

    Eigen::Vector3d e_; //endeffector point expressed in e_frame_

    Eigen::Vector3d v_; //place cylinder axis
    Eigen::Vector3d p_; //place cylinder reference point
    double r_; //place cylinder radius

    Eigen::Vector3d n_; //place plane normal
    double d_; //place plane offsets d !> 0
};
////-----------------------------------------------------------
//struct CartesianStiffness
//{
//    double sx;
//    double sy;
//    double sz;
//    double sa;
//    double sb;
//    double sc;
//};
//-----------------------------------------------------------
class DemoPalletizing
{
public:

    DemoPalletizing();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle n_;
    boost::mutex manipulator_tasks_m_;
    boost::condition_variable cond_;
    double task_error_tol_;
    double task_diff_tol_;
    bool task_status_changed_;
    bool task_success_;
    bool with_gazebo_; ///<indicate whether the node is run in simulation
    std::vector<unsigned int> pers_task_vis_ids_; ///< indicates which persistent tasks (the ones which are loaded) should always be visualized

    //**Grasp definition - this should be modified to grasp different objects */
    GraspInterval grasp_;
    PlaceInterval place_zone_; ///< placement zone for the object
    lbr_fri::SetStiffness cart_stiffness_;
    Eigen::VectorXd t_prog_prev_;

    ros::Subscriber task_status_sub_;
    ros::ServiceClient set_tasks_clt_;
    ros::ServiceClient get_grasp_interval_clt_;
    ros::ServiceClient activate_hqp_control_clt_;
    ros::ServiceClient set_task_objects_clt_;
    ros::ServiceClient visualize_task_geometries_clt_;
    ros::ServiceClient remove_tasks_clt_;
    ros::ServiceClient set_gazebo_physics_clt_;
    ros::ServiceClient velvet_pos_clt_;
    ros::ServiceClient load_tasks_clt_;
    ros::ServiceClient reset_hqp_control_clt_;
    ros::ServiceClient velvet_grasp_clt_;
    ros::ServiceClient set_stiffness_clt_;
    ros::ServiceServer start_demo_srv_;


    //** Manipulator joint configuration from which the demo can be started safely */
    std::vector<double> home_config_;
    //** Manipulator joint configuration while moving the forklift */
    std::vector<double> transfer_config_;
    //** Manipulator joint configuration prior to reach-to-grasp */
    std::vector<double> sensing_config_;
    //** message holding the active tasks at each state. After each state change these tasks are removed and replaced by the ones corresponding to the next state. */
    hqp_controllers_msgs::SetTasks tasks_;
    //** map holding the ids of those tasks whose completion indicates a state change*/
    std::vector<unsigned int> monitored_tasks_;
    // std::map<std::string, hqp_controllers_msgs::TaskObject> task_object_templates_;


    //** To be called before entering a new state*/
    bool resetState();
    
    //** sends the filled SetTasks to the controller*/
    bool sendStateTasks();
    //** visualizes the tasks_*/
    bool visualizeStateTasks(std::vector<unsigned int> const& ids);

    //** deactivates the HQP control scheme - the controller will output zero velocity*/
    void deactivateHQPControl();
    //** activates the HQP control scheme*/
    void activateHQPControl();
    //**First deactivates the HQP control scheme (the controller will output zero velocity commands afterwards) and then calls a ros::shutdown */
    void safeShutdown();

    bool setJointConfiguration(std::vector<double> const& joints);

    bool setGraspApproach();
    bool setObjectExtract();
    bool setObjectTransfer();
    bool setGripperExtract();
    bool setObjectPlace();
    bool loadPersistentTasks();
    bool getGraspInterval();
    bool getPileGraspInterval();

    //double maximumNorm(std::vector<double>const& e);

    //void generateTaskObjectTemplates();

    /////////////////
    //  CALLBACKS  //
    /////////////////

    void stateCallback( const hqp_controllers_msgs::TaskStatusArrayPtr& msg);
    bool startDemo(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res );
};

}//end namespace hqp controllers

#endif



