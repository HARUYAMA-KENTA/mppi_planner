#ifndef MPPI_PATH_MPPI_H
#define MPPI_PATH_MPPI_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <visualization_msgs/Marker.h>

using vec2_t = Eigen::Matrix<double, 2, 1>;
using mat_t = Eigen::Matrix<double, 2, 2>;

// struct Series_eig {
//     std::vector<vec2_t> controls;
//     Series_eig(size_t size) : controls(size, vec2_t::Zero()) {}
// };

class MPPIPlanner {
public:
    MPPIPlanner();
    void calc_optical_control_input();
    int get_nearest_waypoint(double x, double y, bool update_prev_idx);

private:
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void model_callback(const gazebo_msgs::ModelStatesConstPtr &msg);
    void path_callback(const nav_msgs::PathConstPtr &msg);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);
    void target_point_callback(const geometry_msgs::PoseStampedConstPtr &msg);
    bool goal_check();
    // double calc_stage_cost(const Series_eig &v_samples);
    double calc_terminal_cost(const geometry_msgs::Pose &pose);
    double calc_trajectory_cost(const geometry_msgs::Pose &pose);
    // int search_path_index(const geometry_msgs::Pose &pose);
    int search_path_index(const geometry_msgs::Pose &pose ,bool update_prev_idx);

    // std::vector<double> calc_weights(std::vector<Series_eig> V);
    std::vector<double> calc_weights(std::vector<double> &Stagecost);
    std::vector<vec2_t> moving_average(const std::vector<vec2_t>& xx, 
                                        const size_t& window_size);
    // std::vector<Series_eig> random_sampling(const Series_eig &control_input, const mat_t &cov);
    
    vec2_t sample_noise(const vec2_t &mean_vec, const mat_t &cov);
    vec2_t clamp_input(vec2_t &input_control, double &vel_max, double &vel_min, double &omega_lim);
    geometry_msgs::Pose state_transition(geometry_msgs::Pose pose, const vec2_t &v);
    ros::NodeHandle nh_;
    ros::Publisher optim_velocity_pub_;
    ros::Publisher optim_path_pub_;
    ros::Publisher sampled_path_pub_;
    ros::Publisher waypoint_pub;
    ros::Subscriber amcl_sub_;
    ros::Subscriber model_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber target_point_sub_;

    double theta,dt;
    vec2_t ut;
    double max_vel;
    double max_omega;
    double min_vel;
    double lambda,inv_lambda;
    double goal_threshold;
    bool goal_check_flg;
    bool start_call;
    geometry_msgs::Twist optim_control;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose state_pose;
    // geometry_msgs::Pose trajectory_pose;
    geometry_msgs::Pose X;
    geometry_msgs::Pose X_t;
    geometry_msgs::PoseStamped goal_check_pose;
    geometry_msgs::PoseStamped optimal_path;
    nav_msgs::Path goal_path,trajectory_path;
    // nav_msgs::Path goal_path;
    mat_t sigma;
    mat_t inv_sigma;
    double gamma;
    double alpha;
    double window_size;
    int K;
    int time_horizon_T;
    int search_path_window, path_index_size,current_way,current_way_c,current_way_phi;
    std::vector<double> S_cost;
    std::vector<double> weight_;
    std::vector<vec2_t> input_U,optimal_U;
    std::vector<std::vector<vec2_t>> epsilon_;
    std::vector<std::vector<vec2_t>> V;
    std::vector<vec2_t>w_epsilon;
    // Series_eig input_U = Series_eig(time_horizon_T);
    // Series_eig optimal_U = Series_eig(time_horizon_T);
    // Series_eig V_ = Series_eig(time_horizon_T);
    // Series_eig epsilon_ = Series_eig(time_horizon_T);
    // Series_eig w_epsilon = Series_eig(time_horizon_T);
    // std::vector<std::vector<vec2_t>> epsilon_ce;
    // std::vector<Series_eig> Vk;
    std::vector<std::vector<double>> ref_path;
    int prev_waypoints_idx = 0;
};

#endif // MPPI_PATH_MPPI_H
