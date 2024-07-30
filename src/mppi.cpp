
#include "mppi_path/mppi.h"

MPPIPlanner::MPPIPlanner()
{
    // Control_;
    // Series_;
    // Series_eig;
    // Series_eig_vec;
    // ------publisher and subscriber -------
    ROS_INFO("MPPI planner");
    optim_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    optim_path_pub_ = nh_.advertise<nav_msgs::Path>("/optimal_path",1);
    sampled_path_pub_ = nh_.advertise<nav_msgs::Path>("/sampled_path",1);
    waypoint_pub = nh_.advertise<visualization_msgs::Marker>("waypoint",1);
    // odom_sub_ = nh_.subscribe("/odom", 1, &MPPIPlanner::odom_callback, this);
    // amcl_sub_ = nh_.subscribe("/amcl_pose",1,&MPPIPlanner::amcl_callback, this);
    model_sub_ = nh_.subscribe("/gazebo/model_states",1,&MPPIPlanner::model_callback, this);
    // local_map_sub_ = nh_.subscribe("/local_map", 1, &MPPIPlanner::local_map_callback, this);
    // path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan",1,&MPPIPlanner::path_callback, this); 
    path_sub_ = nh_.subscribe("/nav_path",1,&MPPIPlanner::path_callback, this); 
    target_point_sub_ = nh_.subscribe("move_base_simple/goal", 1, &MPPIPlanner::target_point_callback,this);
    // scan_sub_ = nh_.subscribe("/scan", 1, &MPPIPlanner::scan_callback, this);

    //-----set params-----
    // noise param sigma
    sigma << 0.25, 0.0,
             0.0, 0.5;
    inv_sigma = sigma.inverse(); 
    // clip control param
    max_vel = 0.4;
    max_omega = 0.5;
    min_vel = -0.1;
    dt = 0.05;

    lambda = 1.0;
    inv_lambda = 1.0/lambda;
    alpha = 0.2;
    gamma = lambda * (1-alpha);
    start_call = false;
    path_receive_flg = false;
    
    goal_threshold = 0.3;
    goal_check_flg = false;
    path_index_size = 0;
    search_path_window = 100;
    current_way = 0;
    //sample num
    K = 100;
    window_size = 10;
    //step horizon num
    time_horizon_T = 30;
    prev_waypoints_idx = 0;

    //----- init vector----- 
    input_U.resize(time_horizon_T);
    optimal_U.resize(time_horizon_T);
    S_cost.resize(K);
    // weight_.resize(K);
    w_epsilon.resize(K);
    for (int i=0;i<time_horizon_T;i++){
        input_U[i] = vec2_t::Zero(2,1);
    }
    optimal_U = input_U;
    epsilon_.resize(K);
    V.resize(K);
    for (int i=0;i<K;i++){
        epsilon_[i].resize(time_horizon_T);
        V[i].resize(time_horizon_T);
    }
    // goal_path.poses.clear();

}

//---------callback-------------
void MPPIPlanner::amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    current_pose = msg ->pose.pose;
    // ROS_INFO("pose received!!:");

}
void MPPIPlanner::model_callback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    current_pose = msg->pose[2];
    // ROS_INFO("pose received!!:");

}

void MPPIPlanner::path_callback(const nav_msgs::PathConstPtr &msg){
    goal_path.poses = msg->poses;
    path_index_size = goal_path.poses.size();
    ROS_INFO("path call!!:num%d",path_index_size);
    path_receive_flg = true;
    
}

void MPPIPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    //watch later
}
void MPPIPlanner::target_point_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    goal_check_pose.pose = msg->pose;
    ROS_INFO("goal pose received!!:");
    start_call = true;


}
//---------function-------------
//F(Xt,vt) 
geometry_msgs::Pose MPPIPlanner::state_transition(geometry_msgs::Pose pose, const vec2_t &vt){//F(Xt,vt)
    theta = tf2::getYaw(pose.orientation);
    double theta_t;
    double x_t ,y_t;

    // if(vt[1]<0.00001)
    // {
        x_t = pose.position.x + (vt[0] * cos(theta) *dt);// v[0] =v
        y_t = pose.position.y + (vt[0] * sin(theta) *dt);// v[1] = omega
        theta_t = theta + (vt[1]  *dt);
    // }
    // else
    //     x_t = pose.position.x + (vt[0]/(vt[1]*(sin(theta+vt[1]*dt)-sin(theta))));// v[0] =v
    //     y_t = pose.position.y + (vt[0]/(vt[1]*(cos(theta+vt[1]*dt)-cos(theta))));// v[0] =v
    //     theta_t = theta + vt[1] *dt;
    pose.position.x = x_t;
    pose.position.y = y_t;
    pose.position.z = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, theta_t),
                        pose.orientation);
    // std::cout << "x:"<< pose.position.x <<"y:" << pose.position.y << std::endl;
    
    return pose;
}

vec2_t MPPIPlanner::clamp_input(vec2_t &input_control, double &vel_max, double &vel_min, double &omega_lim) {
    input_control[0] = std::max(std::min(input_control[0], vel_max), vel_min);
    input_control[1] = std::max(std::min(input_control[1], omega_lim), (-omega_lim));
    return input_control;

}

vec2_t MPPIPlanner::sample_noise(const vec2_t& mean_vec, const mat_t & cov){
    std::mt19937 engine((std::random_device())());
    std::normal_distribution<> dist(0.0, 1.0);
    Eigen::LLT<Eigen::MatrixXd> llt(cov);
    Eigen::MatrixXd L = llt.matrixL();
    vec2_t n;
    for (size_t i = 0; i < 2; ++i)
    {
        n(i) = dist(engine);
    }
    return L * n; //[v_epsilon,omega_epsilon]
}

// std::vector<std::vector<vec2_t>> MPPIPlanner::random_sampling(const Series_eig&control_input, const mat_t &cov){
//     // Series_eig_vec Vk;
//     std::cout << "U:" <<control_input.controls[0] << std::endl;
//     std::vector<Series_eig> V_(K, Series_eig(time_horizon_T));
    
//     vec2_t v = vec2_t::Zero();
//     for(int k=0;k<K;k++){
//         for(int t=0;t<time_horizon_T;t++){
//             epsilon_ce[k][t] = sample_noise(control_input.controls[t],sigma);
//             v = control_input.controls[t] + epsilon_ce[k][t];
//             V_[k].controls[t] = clamp_input(v, max_vel, min_vel, max_omega);
//             // std::cout <<"V["<< k <<"]"<< V_[k].controls[t]<< std::endl ;
            
//         }
//     }
//     return V_;
// }//return Vt
   


bool MPPIPlanner::goal_check(){
    bool check_flg;
    double dist;
    dist = std::sqrt(std::pow(current_pose.position.x - goal_check_pose.pose.position.x, 2) + 
                    std::pow(current_pose.position.y - goal_check_pose.pose.position.y, 2));
    // std::cout << dist << std::endl;
    if (goal_threshold > dist)
    {
        check_flg = true;
        ROS_INFO("reach GOAL!!!");
    }
    else{
        check_flg = false;
        // ROS_INFO("can't reach GOAL!!!");
    }
    return check_flg;
}
int MPPIPlanner::search_path_index(const geometry_msgs::Pose &pose ,bool update_prev_idx){
        
        // int search_index_len = std::min(50,path_index_size-prev_waypoints_idx);
        int search_index_len = 50;
        // std::cout << "len:" << path_index_size << std::endl;
        int prev_idx = prev_waypoints_idx;
        double dist = 0.0;
        std::vector<double>dist_list;
        dist_list.resize(prev_idx+search_index_len);
        // dist_list.resize(path_index_size);
        // if(path_index_size < search_path_window)
        // {
        //     search_path_window = path_index_size;
        // }
        // for(int i=current_way; i<search_path_window; i++)
    // #pragma omp parallel for schedule(dynamic)    
        for(int i=prev_idx; i<prev_idx+search_index_len; ++i)
        {
            // std::cout << "search:" << i << std::endl;
            double dx = std::pow((pose.position.x - goal_path.poses[i].pose.position.x),2);
            double dy = std::pow((pose.position.y - goal_path.poses[i].pose.position.y),2);            
            // dist = dx  + dy ;
            dist_list[i] = dx + dy;
            // std::cout << "dist_list" << dist_list[i] << std::endl;
        }
    

       
        int min_d = std::distance(dist_list.begin(), 
                std::min_element(dist_list.begin(),dist_list.end()));
        // std::cout << "min_d:" << min_d << std::endl;
        // std::cout << "prev_idx:" << prev_idx << std::endl;
        int nearest_idx = min_d + prev_idx; 
        
        // double ref_x =  goal_path.poses[nearest_idx].pose.position.x;
        // double ref_y =  goal_path.poses[nearest_idx].pose.position.y;
            // if(dist <= old_dist){
            //     old_dist = dist;
            //     near_way_index = i;
            // }

        if(update_prev_idx)
            // nearest_idx =  search_index_len;
            // if(path_index_size < nearest_idx)
            //     nearest_idx = path_index_size;
            prev_waypoints_idx = nearest_idx+1;
            if(path_index_size <= prev_waypoints_idx)
                prev_waypoints_idx = nearest_idx;

            // std::cout <<"update way:"<< prev_waypoints_idx<< std::endl ;
        // std::cout << "len end:" << search_index_len << std::endl;

        return nearest_idx;
    }
double MPPIPlanner::calc_trajectory_cost(const geometry_msgs::Pose& pose)
{
    // int near_way_index_tra = current_way_c;
    double cost_c = 0.0;

    // near_way_index_tra = search_path_index(pose);
    // cost_c = std::sqrt(std::pow(pose.position.x - goal_path.poses[near_way_index_tra].pose.position.x , 2) + 
                    //  std::pow(pose.position.y - goal_path.poses[near_way_index_tra].pose.position.y, 2));
    // cost_c = std::sqrt(std::pow(pose.position.x - goal_check_pose.pose.position.x , 2) + 
                    // std::pow(pose.position.y - goal_check_pose.pose.position.y, 2));
    int near_way_index_c = search_path_index(pose,false);

    cost_c = 1.0*std::pow((pose.position.x - goal_path.poses[near_way_index_c].pose.position.x),2) + 
                2.0*std::pow((pose.position.y - goal_path.poses[near_way_index_c].pose.position.y),2); 
    // cost_c += 1.0*std::pow((pose.position.x - goal_check_pose.pose.position.x),2) + 
                // 1.0*std::pow((pose.position.y - goal_check_pose.pose.position.y),2); 

    // current_way_c = near_way_index_tra;
    return cost_c;
}
double MPPIPlanner::calc_terminal_cost(const geometry_msgs::Pose& pose)
{
    double cost_phi = 0;
    int near_way_index_phi = search_path_index(pose,false);

    cost_phi = 1.0*std::pow((pose.position.x - goal_path.poses[near_way_index_phi].pose.position.x),2) + 
                2.0*std::pow((pose.position.y - goal_path.poses[near_way_index_phi].pose.position.y),2); 

    // cost_phi += 1.0*std::pow((pose.position.x - goal_check_pose.pose.position.x),2) + 
                // 1.0*std::pow((pose.position.y - goal_check_pose.pose.position.y),2); 
    // std::cout << "phi:"<<cost_phi << std::endl;
    // current_way_phi = near_way_index_phi;
    return cost_phi;
}


//Algorithm 2: Information Theoretic Weight Computation 
// std::vector<double> MPPIPlanner::calc_weights(std::vector<Series_eig> V){
std::vector<double> MPPIPlanner::calc_weights(std::vector<double> &Stage_cost){
    // rho = findMinEigenVector()
    std::vector<double> weight;
    weight.resize(K);
    std::vector<nav_msgs::Path> path_list;

    auto min_element = std::min_element(Stage_cost.begin(),Stage_cost.end());
    double rho = *min_element;
    double eta=0;

    // rho = *std::min_element(Stage_cost.begin(),Stage_cost.end());
    // std::cout << "rho:"<< rho << std::endl;
#pragma omp parallel for reduction(+ : eta) schedule(dynamic)
    for(const auto& c:Stage_cost){
    // for(size_t k=0;k<K;++k){
        eta += std::exp(-inv_lambda*(c-rho));

        // eta += std::exp(-inv_lambda * (Stage_cost[k]-rho));
    }
    // std::cout << "eta:"<< eta << std::endl;
    double inv_eta = 1.0/eta;
// #pragma omp parallel for schedule(dynamic)
    // w[] = (1/eta) * std::exp((-1/lambda)*inv_sigma)
    for(size_t k=0;k<K;++k){
        weight[k] = inv_eta * std::exp(-inv_lambda*(Stage_cost[k]-rho));
    
        // std::cout << "weight:"<< weight[k] << std::endl;
    }
    return weight; //vector<double> weight= [wo,w1...wk]
}//return w

std::vector<vec2_t> MPPIPlanner::moving_average(const std::vector<vec2_t>& xx, 
                                                const size_t& window_size) {
    const size_t n = xx.size();
    std::vector<vec2_t> xx_mean(n, vec2_t::Zero(2, 1));
    std::vector<double> window(window_size, 1.0 / window_size);
    
    for (size_t d = 0; d < 2; ++d) {
        std::vector<double> temp(n + window_size - 1, 0.0);
        // Padding the temp array with the first and last values
        for (size_t i = 0; i < n; ++i)
          temp[i + window_size * 0.5] = xx[i](d);
        for (size_t i = 0; i < n; ++i) {
          double sum = 0.0;
          for (size_t j = 0; j < window_size; ++j) {
            sum += temp[i + j] * window[j];
          }
          xx_mean[i](d) = sum;
        }
        size_t n_conv = ceil(window_size / 2.0);
        xx_mean[0](d) *= window_size / n_conv;
        for (size_t i = 1; i < n_conv; ++i) {
          xx_mean[i](d) *= window_size / (i + n_conv);
          xx_mean[n - i - 1](d) *= window_size / (i + n_conv - (window_size % 2));
        }
    }
    return xx_mean;
}
//main function
void MPPIPlanner::calc_optical_control_input(){
//calculate optimal input U
    vec2_t v = vec2_t::Zero();
    trajectory_path.poses.resize(time_horizon_T);
    X_t = current_pose; //set current pose X

    input_U = optimal_U; // set prev control input u
    std::cout << "size" << input_U.size() << std::endl;
    goal_check_flg = goal_check(); 
    // path_index_size = goal_path.poses.size();
  
    visualization_msgs::Marker way_marker;
    way_marker.header.frame_id = "map";
    way_marker.header.stamp = ros::Time::now();
    way_marker.type = visualization_msgs::Marker::CUBE;
    way_marker.ns = "way";
    way_marker.id = 0;
    way_marker.action = visualization_msgs::Marker::ADD;
    way_marker.lifetime = ros::Duration();
    if(start_call && path_receive_flg && !goal_check_flg){
        // std::cout <<  goal_check_flg << std::endl;
        search_path_index(X_t,true);
        std::cout <<"way:"<< prev_waypoints_idx<< std::endl ;
        if(prev_waypoints_idx >= path_index_size){
            std::cout <<"Reached the end of the reference path"<< std::endl ;
        }

        geometry_msgs::PoseStamped tra_pose_;
        nav_msgs::Path sample_path;
        sample_path.header.stamp = ros::Time::now();
        sample_path.header.frame_id = "map";
        sample_path.poses.clear();
        // std::cout << "debug" << std::endl;
        // #pragma omp parallel for schedule(dynamic)
        for(size_t k = 0;k<K;++k){
            X = X_t;
            S_cost[k] = 0.0;
            // current_way_c = current_way;
            
            for(size_t t = 1;t<time_horizon_T+1;++t){
                epsilon_[k][t-1] = sample_noise(input_U[t-1],sigma);
                // std::cout <<"epsilon:" <<k <<":"<<t<<":"<< epsilon_[k][t-1] << std::endl;
                
                if (k < (1-alpha)*K)
                {
                    v = input_U[t-1] + epsilon_[k][t-1];

                }
                else
                    v = epsilon_[k][t-1];
                v = clamp_input(v, max_vel, min_vel, max_omega);
                V[k][t-1] = v;

                X = state_transition(X, v);
                //calculate stage cost c
                S_cost[k] += calc_trajectory_cost(X) + gamma 
                                * input_U[t-1].transpose() *inv_sigma *v;
                
                // geometry_msgs::PoseStamped pose_stamped;
                tra_pose_.header.stamp = ros::Time::now();
                tra_pose_.header.frame_id = "map";
                tra_pose_.pose.position.x = X.position.x;
                tra_pose_.pose.position.y = X.position.y;
                tra_pose_.pose.position.z = 0;
                sample_path.poses.push_back(tra_pose_);
            }
            //calculate terminal cost phi
            S_cost[k] += calc_terminal_cost(X);
            // std::cout<<"t_phi:" <<trajectory_pose << std::endl;
            sampled_path_pub_.publish(sample_path);
            // std::cout <<"S["<< k <<"]"<< S[k]<< std::endl ;
        }

        weight_ = calc_weights(S_cost);
        // search_path_index(X_t,true);

        // current_way = current_way_phi;
        way_marker.pose.position.x = goal_path.poses[prev_waypoints_idx].pose.position.x;
        way_marker.pose.position.y = goal_path.poses[prev_waypoints_idx].pose.position.y;
        // way_marker.pose.position.x = goal_check_pose.pose.position.x;
        // way_marker.pose.position.y = goal_check_pose.pose.position.y;
        way_marker.pose.position.z = 0.0;
        way_marker.pose.orientation.x = 0;
        way_marker.pose.orientation.y = 0;
        way_marker.pose.orientation.z = 0;
        way_marker.pose.orientation.w = 1;
        way_marker.color.r  = 0.0;
        way_marker.color.g  = 0.0;
        way_marker.color.b  = 1.0;
        way_marker.color.a  = 1.0;

        way_marker.scale.x = 0.1;
        way_marker.scale.y = 0.1;
        way_marker.scale.z = 0.1;
        waypoint_pub.publish(way_marker);


        // std::cout << "optim" << std::endl;
        std::vector<geometry_msgs::PoseStamped> u_path;
        geometry_msgs::PoseStamped optim_pose;
        nav_msgs::Path visual_path;
    // #pragma omp parallel for schedule(dynamic)
        for(int t = 0;t<time_horizon_T;++t){
            w_epsilon[t] = vec2_t::Zero(2,1);
            for(int k = 0;k<K;++k){
                w_epsilon[t] += weight_[k] * epsilon_[k][t];
            }
        }
        w_epsilon = moving_average(w_epsilon,window_size=10);
        geometry_msgs::PoseStamped pose_stamped;
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        path.poses.clear();
        //update X pose
        X = X_t;

        for (size_t t = 0; t< time_horizon_T; ++t){
            input_U[t] += w_epsilon[t];
            input_U[t] = clamp_input(input_U[t], max_vel, min_vel, max_omega);
            X = state_transition(X,input_U[t]);
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = X.position.x;
            pose_stamped.pose.position.y = X.position.y;
            pose_stamped.pose.position.z = 0;
            path.poses.push_back(pose_stamped);
        }
        size_t U_N = input_U.size();
        std::copy(input_U.begin() + 1, input_U.end(), optimal_U.begin());
        optimal_U[U_N-1] = input_U[U_N-1];
        optim_control.linear.x = input_U[0][0];
        optim_control.angular.z = input_U[0][1];
        // std::cout << "pub:" <<input_U[0][0] <<input_U[0][1]<< std::endl;
        optim_path_pub_.publish(path);
        optim_velocity_pub_.publish(optim_control);


    }
    else{
        std::cout << "please start call or goal now" << std::endl;
        optim_control.linear.x =  0.0;
        optim_control.angular.z = 0.0;
        for (int i=0;i<time_horizon_T;i++){
            input_U[i] = vec2_t::Zero(2,1);
        }   
        optim_velocity_pub_.publish(optim_control);
    }
}
