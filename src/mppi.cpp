#include "mppi_path/mppi.h"


MPPIPlanner::MPPIPlanner()
{
    ROS_INFO("MPPI planner");

    optim_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/odom", 1, &MPPIPlanner::odom_callback, this);
    local_map_sub_ = nh_.subscribe("/local_map", 1, &MPPIPlanner::local_map_callback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &MPPIPlanner::scan_callback, this);
    // for noise epsilon
    // sigma << 0.5, 0.0, 0.0,
    //          0.0, 0.5, 0.0,
    //          0.0, 0.0, 0.5;
    sigma << 0.5, 0.0,
             0.0, 0.5;
    inv_sigma = sigma.inverse(); 
}
//---------callback-------------
void MPPIPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    current_pose = msg ->pose.pose;
}
//---------function-------------
void MPPIPlanner::state_transition(double sample_vt,double omega){//F
    theta = tf2::getYaw(current_pose.orientation);
    double x_t = current_pose.position.x + sample_vt * cos(theta) *dt;
    double y_t = current_pose.position.y + sample_vt * sin(theta) *dt;
    double theta_t = tf2::getYaw(current_pose.orientation) + omega *dt;
}
void MPPIPlanner::random_sampling(){
}
void MPPIPlanner::calc_stage_cost(){
}
void MPPIPlanner::calc_terminal_cost(){
}
void MPPIPlanner::calc_trajectory_cost(){
}
void MPPIPlanner::calc_weights(){
}
void MPPIPlanner::calc_optical_control_input(){
}
void MPPIPlanner::run(){
    random_sampling();
    
}


