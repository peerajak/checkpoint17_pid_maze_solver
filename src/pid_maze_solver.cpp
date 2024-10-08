#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>

#define pi 3.1417

using Eigen::MatrixXd;
using namespace std::chrono_literals;
using std::placeholders::_1;
// std::chrono::nanoseconds fifty_milisec = 5000000;

enum SceneType { Simulation = 1, Cyberworld = 2 };

class MazeSolver : public rclcpp::Node {
public:
  MazeSolver(enum SceneType scene_number, bool is_forward_direction)
      : Node("maze_solver"), scene_number_(scene_number),
        is_forward_direction_(is_forward_direction) {
    //  std::string  KP_str = argv[1];
    //  std::string  KI_str = argv[2];
    //  std::string  KD_str = argv[3];
    //  std::string  KP_angle_str = argv[4];
    //  std::string  KI_angle_str = argv[5];
    //  std::string  KD_angle_str = argv[6];
    // ---- 1. publisher to cmd_vel
    publisher_1_twist =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //------- 2. timer_1 related -----------//
    timer_1_ =
        this->create_wall_timer(2000ms, std::bind(&MazeSolver::timer1_callback,
                                                  this)); // nothing about 1 sec

    // ref_points->push_back(std::make_tuple(0,0));
    double cur_ref_phi = 0;
    double cur_ref_x = 0;
    double cur_ref_y = 0;
    // RCLCPP_DEBUG(this->get_logger(), "initialize ref_point (x,y) = %f,%f ",
    // cur_ref_x, cur_ref_y);

    // PID parameter   best Kp=4.0
    switch (this->scene_number_) {
    case Simulation:
      this->Kp = 1.4;
      this->Ki = 0.05;
      this->Kd = 0;
      this->Kp_angle = 1.2;
      this->Ki_angle = 0.005;
      this->Kd_angle = 0.005;
      RCLCPP_DEBUG(
          this->get_logger(),
          "Simulation "
          "Scene,Kp:%f,Ki:%f,Kd:%f,Kp_angle:%f,Ki_angle:%f,Kd_angle:%f",
          this->Kp, this->Ki, this->Kd, this->Kp_angle, this->Ki_angle,
          this->Kd_angle);
      if (!this->is_forward_direction_) {
        std::reverse(this->ref_points_simulation.begin(),
                     this->ref_points_simulation.end());
      }
      break;
    case Cyberworld:
      this->Kp = 0.5;
      this->Ki = 0;
      this->Kd = 0;
      this->Kp_angle = 0.5; // std::stod(KP_str); // 5.8;//5.5,0,0 best, 5.8 max
                            // . (5.8,0.1,0.001) best
      this->Ki_angle = 0; // std::stod(KI_str); // 1.0;
      this->Kd_angle = 0; // std::stod(KD_str); // // 0.0;
      RCLCPP_DEBUG(
          this->get_logger(),
          "Cyberworld "
          "Scene,Kp:%f,Ki:%f,Kd:%f,Kp_angle:%f,Ki_angle:%f,Kd_angle:%f",
          this->Kp, this->Ki, this->Kd, this->Kp_angle, this->Ki_angle,
          this->Kd_angle);
      if (!this->is_forward_direction_) {
        std::reverse(this->ref_points_cyberworld.begin(),
                     this->ref_points_cyberworld.end());
      }
      break;
    }
    this->Hz = 100.0;
    this->dt = 0.01;
    this->hz_inverse_us = 10000; // 10 Hz = 0.1 sec = 100,000 microsec
    this->pid_reset();

    //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        // "/rosbot_xl_base_controller/odom", 10,
        std::bind(&MazeSolver::odom_callback, this, std::placeholders::_1),
        options3_odom);
  }

private:
  // int max_iter = 20;

  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer Callback ");
    this->timer_1_->cancel();
    // assert(false);
    std::thread{std::bind(&MazeSolver::execute, this)}.detach();
  }

  void pid_reset() {
    this->integral_x_pos = 0;
    this->integral_y_pos = 0;
    this->integral_theta = 0;
    this->last_error_signal = std::make_tuple(0, 0, 0);
  }
  // PID FUNCTIONS

  std::tuple<double, double, double>
  pid_controller(std::tuple<double, double, double> &error_signal) {
    double dtheta_pos = std::get<0>(error_signal);
    double dx_pos = std::get<1>(error_signal);
    double dy_pos = std::get<2>(error_signal);
    RCLCPP_DEBUG(get_logger(),
                 "pid_controller dx_pos %f, dy_pos %f,dtheta_pos %f", dx_pos,
                 dy_pos, dtheta_pos);
    double omega = dtheta_pos / this->dt; //-current_speed_.angular.z;
    double vx = dx_pos / this->dt;        //-current_speed_.linear.x;//
    double vy = dy_pos / this->dt;        //-current_speed_.linear.y;//
    this->integral_x_pos += dx_pos * this->dt;
    this->integral_y_pos += dy_pos * this->dt;
    this->integral_theta += dtheta_pos * this->dt;
    double proportion_signal_x = this->Kp * dx_pos;
    double proportion_signal_y = this->Kp * dy_pos;
    double proportion_signal_theta = this->Kp_angle * dtheta_pos;
    double integral_signal_x = this->Ki * this->integral_x_pos;
    double integral_signal_y = this->Ki * this->integral_y_pos;
    double integral_signal_theta = this->Ki_angle * this->integral_theta;
    double derivative_signal_x =
        this->Kd * (dx_pos - std::get<1>(this->last_error_signal)) /
        this->dt; // this->Kd * vx;
    double derivative_signal_y =
        this->Kd * (dy_pos - std::get<2>(this->last_error_signal)) /
        this->dt; // this->Kd * vy;
    double derivative_signal_theta =
        this->Kd_angle * (dtheta_pos - std::get<0>(this->last_error_signal)) /
        this->dt; // this->Kd_angle * omega;
    omega = normalize_angle(proportion_signal_theta + integral_signal_theta +
                            derivative_signal_theta);
    vx = proportion_signal_x + integral_signal_x + derivative_signal_x;
    vy = proportion_signal_y + integral_signal_y + derivative_signal_y;
    this->last_error_signal = error_signal;
    std::tuple<double, double, double> controller_signal =
        std::make_tuple(omega, vx, vy);
    return controller_signal;
  }

  std::tuple<double, double, double>
  pid_plant_process(std::tuple<double, double, double> &controller_signal) {
    auto message = std_msgs::msg::Float32MultiArray();
    double w = std::get<0>(controller_signal);
    double delta_x = std::get<1>(controller_signal);
    double delta_y = std::get<2>(controller_signal);
    MatrixXd vb = velocity2twist(w, delta_x, delta_y);
    std::vector<float> u_vector = twist2wheels(vb);
    message.data = u_vector;
    wheels2ling(message);
    std::tuple<double, double, double> output_signal =
        std::make_tuple(current_yaw_rad_, current_pos_.x,
                        current_pos_.y); //(theta_pos,x_pos,y_pos)
    return output_signal;
  }

  std::tuple<double, double, double>
  pid_error_moving(std::tuple<double, double, double> &output_signal, double xg,
                   double yg) {
    double theta_pos = std::get<0>(output_signal);
    double x_pos = std::get<1>(output_signal);
    double y_pos = std::get<2>(output_signal);
    double thetag = atan2(yg - y_pos, xg - x_pos);
    double theta_error = normalize_angle(thetag - theta_pos);
    double delta_x_world_coordinate = xg - x_pos;
    double delta_y_world_coordinate = yg - y_pos;
    double delta_x_robot_coordinate =
        cos(theta_error) * delta_x_world_coordinate -
        sin(theta_error) * delta_y_world_coordinate;
    double delta_y_robot_coordinate =
        sin(theta_error) * delta_x_world_coordinate +
        cos(theta_error) * delta_y_world_coordinate;
    std::tuple<double, double, double> error_signal = std::make_tuple(
        theta_error, delta_x_robot_coordinate, delta_y_robot_coordinate);
    RCLCPP_DEBUG(get_logger(),
                 "moving output "
                 "|Goal_x:%f,Goal_y:%f|thetag:%f|Current_x:%f,Current_y:%f|"
                 "current_yaw:%f|angular error:%f ",
                 xg, yg, thetag, x_pos, y_pos, theta_pos, theta_error);
    return error_signal;
  }

  std::tuple<double, double, double>
  pid_error_rotating(std::tuple<double, double, double> &output_signal,
                     double xf, double yf) {
    double theta_pos = std::get<0>(output_signal);
    double x_pos = std::get<1>(output_signal);
    double y_pos = std::get<2>(output_signal);
    double thetag = atan2(yf - y_pos, xf - x_pos);
    double theta_error = normalize_angle(thetag - theta_pos);
    std::tuple<double, double, double> error_signal =
        std::make_tuple(theta_error, 0, 0);
    RCLCPP_DEBUG(get_logger(),
                 "rotating output "
                 "|Goal_x:%f,Goal_y:%f|thetag:%f|Current_x:%f,Current_y:%f|"
                 "current_yaw:%f|angular error:%f ",
                 xf, yf, thetag, x_pos, y_pos, theta_pos, theta_error);
    return error_signal;
  }

  bool pid_simulate_moving(double x_goal, double y_goal, double tolerance,
                           double angle_tolerance) {
    // double theta_goal = normalize_angle(theta_goal_radian);
    RCLCPP_DEBUG(get_logger(), "x_goal %f, y_goal %f, current postion (%f,%f)",
                 x_goal, y_goal, current_pos_.x, current_pos_.y);

    double distance_error_norm = 1000; // some large number
    double error_angle = 1000;         // some large number
    int number_of_secs = 20;
    int time_to_move = this->Hz * number_of_secs;
    // bool historical_distance_error[10] =
    // {false;false;false;false;false;false;false;false;false;false};
    int within_tolerance_counter = 0;
    bool is_stabilized_success = false;

    for (int i = 0; i < time_to_move; i++) {
      // while(distance_error_norm > tolerance ||
      // fabs(normalize_angle(error_angle)) > angle_tolerance){
      std::tuple<double, double, double> output_signal =
          std::make_tuple(current_yaw_rad_, current_pos_.x,
                          current_pos_.y); //(theta_pos,x_pos,y_pos)
      std::tuple<double, double, double> error =
          pid_error_moving(output_signal, x_goal, y_goal);
      error_angle = std::get<0>(error);
      double error_x = std::get<1>(error);
      double error_y = std::get<2>(error);
      RCLCPP_DEBUG(this->get_logger(),
                   "error_angle= %f, error_x= %f,error_y=%f", error_angle,
                   error_x, error_y);
      std::tuple<double, double, double> res =
          pid_controller(error);              //(omega, vx, vy)
      output_signal = pid_plant_process(res); //(theta_pos,x_pos,y_pos)
      distance_error_norm = sqrt(error_x * error_x + error_y * error_y);

      if (distance_error_norm <= tolerance) {
        RCLCPP_DEBUG(this->get_logger(),
                     "moving distance_error_norm= %f, error_angle= %f, "
                     "distance in tolerence ",
                     distance_error_norm, error_angle);
        within_tolerance_counter++;
        if (within_tolerance_counter >= 10) {
          is_stabilized_success = true;
          break;
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "moving distance_error_norm= %f, error_angle= %f ",
                     distance_error_norm, error_angle);
        within_tolerance_counter = 0;
      }
      usleep(hz_inverse_us);
    }
    this->pid_reset();
    return is_stabilized_success;
  }

  bool pid_simulate_rotating(double x_goal, double y_goal, double tolerance,
                             double angle_tolerance) {
    // double theta_goal = normalize_angle(theta_goal_radian);
    RCLCPP_DEBUG(get_logger(), "x_goal %f, y_goal %f", x_goal, y_goal);

    double distance_error_norm = 1000; // some large number
    double error_angle = 1000;         // some large number
    int number_of_secs = 20;
    int time_to_move = this->Hz * number_of_secs;
    // bool historical_distance_error[10] =
    // {false;false;false;false;false;false;false;false;false;false};
    int within_tolerance_counter = 0;
    bool is_stabilized_success = false;

    for (int i = 0; i < time_to_move; i++) {
      // while(distance_error_norm > tolerance ||
      // fabs(normalize_angle(error_angle)) > angle_tolerance){
      std::tuple<double, double, double> output_signal =
          std::make_tuple(current_yaw_rad_, current_pos_.x,
                          current_pos_.y); //(theta_pos,x_pos,y_pos)
      std::tuple<double, double, double> error =
          pid_error_rotating(output_signal, x_goal, y_goal);
      error_angle = std::get<0>(error);
      double error_x = std::get<1>(error);
      double error_y = std::get<2>(error);
      RCLCPP_DEBUG(this->get_logger(),
                   "error_angle= %f, error_x= %f,error_y=%f", error_angle,
                   error_x, error_y);
      std::tuple<double, double, double> res =
          pid_controller(error);              //(omega, vx, vy)
      output_signal = pid_plant_process(res); //(theta_pos,x_pos,y_pos)
      distance_error_norm = sqrt(error_x * error_x + error_y * error_y);

      if (fabs(normalize_angle(error_angle)) <= angle_tolerance) {
        RCLCPP_DEBUG(this->get_logger(),
                     "rotating distance_error_norm= %f, error_angle= %f, angle "
                     "in tolerence %f",
                     distance_error_norm, error_angle, angle_tolerance);
        within_tolerance_counter++;
        if (within_tolerance_counter >= 10) {
          is_stabilized_success = true;
          break;
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "rotating distance_error_norm= %f, error_angle= %f ",
                     distance_error_norm, error_angle);
        within_tolerance_counter = 0;
      }
      usleep(hz_inverse_us);
    }
    this->pid_reset();
    return is_stabilized_success;
  }

  void execute() {
    auto message = std_msgs::msg::Float32MultiArray();
    double distance_error_tolerance = 0.01;
    double angle_tolerance = 0.01;
    long int total_elapsed_time = 0;
    bool all_success = true, all_success_rotating = true;
    double prev_xg = 0;
    double prev_yg = 0;
    std::list<std::tuple<double, double, int>> *ref_points;
    switch (this->scene_number_) {
    case Simulation:
      ref_points = &this->ref_points_simulation;
      //   RCLCPP_DEBUG(this->get_logger(), "ref_points Simulation Scene");
      break;
    case Cyberworld:
      ref_points = &this->ref_points_cyberworld;
      // RCLCPP_DEBUG(this->get_logger(), "ref_points Cyberworld Scence");
      break;
    }
    for (auto it2 = std::next(ref_points->begin(), 1); it2 != ref_points->end();
         it2++) {
      double xg = std::get<0>(*it2);
      double yg = std::get<1>(*it2);
      int w_name = std::get<2>(*it2);

      std::string result_pid;
      RCLCPP_DEBUG(this->get_logger(), "next ref_points w%d  (%f,%f)", w_name,
                   xg, yg);

      std::string result_pid_rotating;
      RCLCPP_DEBUG(this->get_logger(),
                   "start face ref_points w%d  (%f,%f), current_yaw_rad %f: %s",
                   w_name, xg, yg, this->current_yaw_rad_,
                   result_pid_rotating.c_str());
      auto beg_rotating = std::chrono::high_resolution_clock::now();
      bool success_rotating = pid_simulate_rotating(
          xg, yg, distance_error_tolerance, angle_tolerance);
      auto end_rotating = std::chrono::high_resolution_clock::now();
      auto duration_rotating =
          std::chrono::duration_cast<std::chrono::microseconds>(end_rotating -
                                                                beg_rotating);
      all_success_rotating = all_success_rotating && success_rotating;
      if (success_rotating) {
        result_pid_rotating = "success";
      } else {
        result_pid_rotating = "failed";
      }
      ling.angular.z = 0;
      ling.linear.x = 0;
      ling.linear.y = 0;
      this->move_robot(ling);
      RCLCPP_DEBUG(this->get_logger(),
                   "end face ref_points w%d  (%f,%f), current_yaw_rad %f: %s, "
                   "elasped time %ld",
                   w_name, xg, yg, this->current_yaw_rad_,
                   result_pid_rotating.c_str(), duration_rotating.count());
      total_elapsed_time += duration_rotating.count();
      RCLCPP_DEBUG(this->get_logger(),
                   "start move ref_points w%d  (%f,%f), current_yaw_rad %f: %s",
                   w_name, xg, yg, this->current_yaw_rad_,
                   result_pid_rotating.c_str());

      auto beg = std::chrono::high_resolution_clock::now();
      bool success = pid_simulate_moving(xg, yg, distance_error_tolerance,
                                         angle_tolerance);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
      all_success = all_success && success;
      if (success) {
        result_pid = "success";
      } else {
        result_pid = "failed";
      }
      ling.angular.z = 0;
      ling.linear.x = 0;
      ling.linear.y = 0;
      this->move_robot(ling);
      RCLCPP_DEBUG(this->get_logger(),
                   "end move ref_points w%d  (%f,%f), current_yaw_rad %f: %s, "
                   "elasped time %ld",
                   w_name, xg, yg, this->current_yaw_rad_, result_pid.c_str(),
                   duration.count());
      total_elapsed_time += duration.count();
      prev_xg = xg;
      prev_yg = yg;

      // int sleep_sec = 3;
      // RCLCPP_DEBUG(this->get_logger(), "Sleep %d secs", sleep_sec);
      // sleep(sleep_sec);
      usleep(hz_inverse_us);
    }
    char all_success_char = all_success ? 'Y' : 'N';
    RCLCPP_DEBUG(
        get_logger(),
        "Summary Kp:%f,Ki:%f,Kd:%f,Kp_angle:%f,Ki_angle:%f,Kd_angle:%f,total "
        "elapsed time:%ld,all successes:%c",
        this->Kp, this->Ki, this->Kd, this->Kp_angle, this->Ki_angle,
        this->Kd_angle, total_elapsed_time, all_success_char);
    rclcpp::shutdown();
  }

  MatrixXd velocity2twist(double dphi, double dx, double dy) {
    RCLCPP_DEBUG(get_logger(), "velocity2twist current_yaw_rad_ %f",
                 current_yaw_rad_);
    MatrixXd R(3, 3);
    R(0, 0) = 1;
    R(0, 1) = 0;
    R(0, 2) = 0;
    R(1, 0) = 0;
    R(1, 1) = cos(current_yaw_rad_);
    R(1, 2) = sin(current_yaw_rad_);
    R(2, 0) = 0;
    R(2, 1) = -sin(current_yaw_rad_);
    R(2, 2) = cos(current_yaw_rad_);
    MatrixXd v(3, 1);
    v(0, 0) = dphi;
    v(1, 0) = dx;
    v(2, 0) = dy;

    MatrixXd twist = R * v;
    return twist;
  }

  std::vector<float> twist2wheels(MatrixXd twist) {
    std::vector<float> u_vector;

    MatrixXd H(4, 3);
    H(0, 0) = (-l - w) / r;
    H(0, 1) = 1 / r;
    H(0, 2) = -1 / r;
    H(1, 0) = (l + w) / r;
    H(1, 1) = 1 / r;
    H(1, 2) = 1 / r;
    H(2, 0) = (l + w) / r;
    H(2, 1) = 1 / r;
    H(2, 2) = -1 / r;
    H(3, 0) = (-l - w) / r;
    H(3, 1) = 1 / r;
    H(3, 2) = 1 / r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0, 0));
    u_vector.push_back(u(1, 0));
    u_vector.push_back(u(2, 0));
    u_vector.push_back(u(3, 0));
    return u_vector;
  }
  std::vector<float> twist2wheels(double wz, double vx, double vy) {
    std::vector<float> u_vector;
    MatrixXd twist(3, 1);
    twist(0, 0) = wz;
    twist(1, 0) = vx;
    twist(2, 0) = vy;
    MatrixXd H(4, 3);
    H(0, 0) = (-l - w) / r;
    H(0, 1) = 1 / r;
    H(0, 2) = -1 / r;
    H(1, 0) = (l + w) / r;
    H(1, 1) = 1 / r;
    H(1, 2) = 1 / r;
    H(2, 0) = (l + w) / r;
    H(2, 1) = 1 / r;
    H(2, 2) = -1 / r;
    H(3, 0) = (-l - w) / r;
    H(3, 1) = 1 / r;
    H(3, 2) = 1 / r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0, 0));
    u_vector.push_back(u(1, 0));
    u_vector.push_back(u(2, 0));
    u_vector.push_back(u(3, 0));
    return u_vector;
  }

  double normalize_angle(double angle_radian) {
    double sign_multiplier, normalized_angle;

    if (angle_radian >= -pi && angle_radian <= pi)
      return angle_radian;

    if (angle_radian < -pi)
      sign_multiplier = 1;

    if (angle_radian > pi)
      sign_multiplier = -1;

    normalized_angle = angle_radian;
    for (int i = 0; i < 20; i++) {
      normalized_angle += sign_multiplier * 2 * pi;
      if (normalized_angle >= -pi && normalized_angle <= pi)
        return normalized_angle;
    }
    return -100;
  }

  MatrixXd KinematicLeastSquareNormalEq(MatrixXd &u) {
    MatrixXd H(4, 3);
    H(0, 0) = (-l - w) / r;
    H(0, 1) = 1 / r;
    H(0, 2) = -1 / r;
    H(1, 0) = (l + w) / r;
    H(1, 1) = 1 / r;
    H(1, 2) = 1 / r;
    H(2, 0) = (l + w) / r;
    H(2, 1) = 1 / r;
    H(2, 2) = -1 / r;
    H(3, 0) = (-l - w) / r;
    H(3, 1) = 1 / r;
    H(3, 2) = 1 / r;
    MatrixXd HTH_inv = (H.transpose() * H).inverse();
    MatrixXd HTHinv_least_square = HTH_inv * H.transpose();
    MatrixXd twist = HTHinv_least_square * u;
    return twist;
  }

  void wheels2ling(const std_msgs::msg::Float32MultiArray msg) {
    MatrixXd u(4, 1);

    RCLCPP_DEBUG(this->get_logger(), "wheel_speed topic callback");
    for (unsigned int i = 0; i < msg.data.size(); i++) {
      u(i, 0) = msg.data[i];
      RCLCPP_DEBUG(this->get_logger(), "I heard: '%f'", u(i, 0));
    }
    MatrixXd twist = KinematicLeastSquareNormalEq(u);
    RCLCPP_DEBUG(this->get_logger(), "twist wz: %f vx ,%f, vy %f", twist(0, 0),
                 twist(1, 0), twist(2, 0));
    ling.angular.z = twist(0, 0);
    ling.linear.x = twist(1, 0);
    ling.linear.y = twist(2, 0);

    this->move_robot(ling);
  }

  void move_robot(geometry_msgs::msg::Twist &msg) {
    publisher_1_twist->publish(msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  geometry_msgs::msg::Twist ling;
  //------- 3. Odom related  Functions -----------//
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    this->current_pos_ = msg->pose.pose.position;
    this->current_angle_ = msg->pose.pose.orientation;
    this->current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    this->current_speed_ = msg->twist.twist;
    /*this->world_est_current_yaw_rad_= current_yaw_rad_ *2;
    double theta_diff = prev_world_est_current_yaw_rad_ -
    world_est_current_yaw_rad_; double delta_x_robot_coordinate = current_pos_.x
    - prev_pos_x; double delta_y_robot_coordinate = current_pos_.y - prev_pos_y;
    double delta_x_world_coordinate =  cos(theta_diff) *
    delta_x_robot_coordinate - sin(theta_diff) * delta_y_robot_coordinate;
    double delta_y_world_coordinate =  sin(theta_diff) *
    delta_x_robot_coordinate + cos(theta_diff) * delta_y_robot_coordinate;
    this->world_est_current_pos_x_ +=  delta_x_world_coordinate;
    this->world_est_current_pos_y_ +=  delta_y_world_coordinate; */

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f']",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
    // this->prev_pos_x = current_pos_.x;
    // this->prev_pos_y = current_pos_.y;
    // this->prev_world_est_current_yaw_rad_ = world_est_current_yaw_rad_;
  }
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }

  //------- 3. Odom related private variables  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  geometry_msgs::msg::Twist current_speed_;
  double current_yaw_rad_;
  double world_est_current_yaw_rad_ = 0;
  double world_est_current_pos_x_ = 0;
  double world_est_current_pos_y_ = 0;
  double prev_pos_x, prev_pos_y, prev_world_est_current_yaw_rad_;
  //--------  Kinematic related private variables --------//
  double l = 0.500 / 2;
  double r = 0.254 / 2;
  double w = 0.548 / 2;
  //------- Pid variables -------//
  double Kp, Ki, Kd, Kp_angle, Ki_angle, Kd_angle, integral_x_pos,
      integral_y_pos, integral_theta, Hz, dt;
  int hz_inverse_us;
  std::tuple<double, double, double> last_error_signal;

  //   std::list<std::tuple<double, double, double>> waypoints
  //   {std::make_tuple(0,1,-1),std::make_tuple(0,1,1),
  //                                 std::make_tuple(0,1,1),std::make_tuple(1.5708,
  //                                 1, -1),std::make_tuple(-3.1415, -1, -1),
  //                                 std::make_tuple(0.0, -1,
  //                                 1),std::make_tuple(0.0, -1,
  //                                 1),std::make_tuple(0.0, -1, -1)};
  std::list<std::tuple<double, double, int>> ref_points_simulation{
      //(x, y, point_name)
      std::make_tuple(0, 0, 0),    // same
      std::make_tuple(0.48, 0, 1), // same
      std::make_tuple(
          0.5223066137364896, -1.3412349495511038,
          2), //(0.48,0,-0.74216),'(1.495466','-0.929541','-0.742126'
      std::make_tuple(1.0845613669732483, -1.3931992411756675, 3),
      std::make_tuple(1.0395751265563804, -0.8338197862237189, 4),
      std::make_tuple(1.4534436494017255, -0.8494273283260394, 5),
      std::make_tuple(1.4196549222698924, -0.29199996173774334, 6),
      std::make_tuple(2.0396605282345055, -0.25722763994174136, 7),
      std::make_tuple(1.9877302391929643, 0.5694935878197983, 8),
      std::make_tuple(1.5990640106450607, 0.5531625432022931, 9),
      std::make_tuple(1.5521891067753861, 0.18596023078249008, 10),
      std::make_tuple(1.033156713350545, 0.14619280521472142, 11),
      std::make_tuple(0.6408862225835807, 0.4888058306883233, 12),
      std::make_tuple(0.10750566381487081, 0.48461071303469294, 13)};

  std::list<std::tuple<double, double, int>> ref_points_cyberworld{
      //(x, y, point_name)
      std::make_tuple(0, 0, 1),
      std::make_tuple(1.8477, 0, 2),
      std::make_tuple(1.8477, -0.47, 3),
      std::make_tuple(1.4612351966850805, -0.47337361981444936, 4),
      std::make_tuple(1.488915913655926, -0.9898359779111149, 5),
      std::make_tuple(1.8784205168310057, -0.9465217795498742, 6),
      std::make_tuple(1.8988494899765054, -1.5221456908917477, 7),
      std::make_tuple(1.472192559660402, -1.5119025143073208, 8),
      std::make_tuple(1.45492122831896, -1.8623709691792418, 9),
      std::make_tuple(0.9932646068019697, -1.8797840080510675, 10),
      std::make_tuple(0.9023012336595384, -0.9874025672370495, 11),
      std::make_tuple(0.47553288048572817, -1.0438755873689844, 12),
      std::make_tuple(0.43925411993242325, -0.543478297043073, 13),
      std::make_tuple(0.06374232769705736, -0.5498248668009166, 14)};

  std::list<std::tuple<double, double, int>> standard_ref_points_cyberworld{
      std::make_tuple(0, 0, 1),
      std::make_tuple(1.8477, 0, 2),
      std::make_tuple(1.8477, -0.47, 3),
      std::make_tuple(1.4612351966850805, -0.47337361981444936, 4),
      std::make_tuple(1.488915913655926, -0.9898359779111149, 5),
      std::make_tuple(1.8784205168310057, -0.9465217795498742, 6),
      std::make_tuple(1.8988494899765054, -1.5221456908917477, 7),
      std::make_tuple(1.472192559660402, -1.5119025143073208, 8),
      std::make_tuple(1.45492122831896, -1.8623709691792418, 9),
      std::make_tuple(0.9932646068019697, -1.8797840080510675, 10),
      std::make_tuple(0.9023012336595384, -0.9874025672370495, 11),
      std::make_tuple(0.47553288048572817, -1.0438755873689844, 12),
      std::make_tuple(0.43925411993242325, -0.543478297043073, 13),
      std::make_tuple(0.06374232769705736, -0.5498248668009166, 14)};

  rclcpp::TimerBase::SharedPtr timer_1_;
  int timer1_counter;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  enum SceneType scene_number_;
  bool is_forward_direction_ = true;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  enum SceneType scene_number = Simulation;
  // Check if a scene number argument is provided
  int scene_number_direction;
  bool is_forward_direction = true;
  if (argc > 1) {
    scene_number_direction = std::atoi(argv[1]);
    is_forward_direction = scene_number_direction > 0;
    scene_number = static_cast<enum SceneType>(abs(scene_number_direction));
  }
  auto maze_solver_node =
      std::make_shared<MazeSolver>(scene_number, is_forward_direction);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(maze_solver_node);
  executor.spin();

  // rclcpp::spin(std::make_shared<MazeSolver>());
  rclcpp::shutdown();
  return 0;
}