#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <memory>
#include <tuple>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


#define pi 3.1417
 

using namespace std::chrono_literals;
using std::placeholders::_1;
//std::chrono::nanoseconds fifty_milisec = 5000000;




class Odometry : public rclcpp::Node {
public:

    Odometry() : Node("maze_solver"){

        //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    // "/rosbot_xl_base_controller/odom", 10,
        std::bind(&Odometry::odom_callback, this,
                  std::placeholders::_1), options3_odom);

  }

private:


  //------- 3. Odom related  Functions -----------//  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {



    this->current_pos_ = msg->pose.pose.position;
    this->current_angle_ = msg->pose.pose.orientation;
    this->current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) - offset_yawrad;
    if(is_first_odom_callback){
    offset_yawrad =  this->current_yaw_rad_ ;
    is_first_odom_callback = false;
    }
        
    
    this->current_speed_ = msg->twist.twist;
    this->world_est_current_yaw_rad_= current_yaw_rad_ *2;
    //double theta_diff = prev_world_est_current_yaw_rad_ - world_est_current_yaw_rad_;
    double delta_x_robot_coordinate = current_pos_.x - prev_pos_x;
    double delta_y_robot_coordinate = current_pos_.y - prev_pos_y;
    double delta_x_world_coordinate =  cos(-world_est_current_yaw_rad_) *  delta_x_robot_coordinate - sin(-world_est_current_yaw_rad_) * delta_y_robot_coordinate;
    double delta_y_world_coordinate =  sin(-world_est_current_yaw_rad_) *  delta_x_robot_coordinate + cos(-world_est_current_yaw_rad_) * delta_y_robot_coordinate; 
    this->world_est_current_pos_x_ +=  delta_x_world_coordinate;
    this->world_est_current_pos_y_ +=  delta_y_world_coordinate; 

    RCLCPP_INFO(this->get_logger(), "current pos=['%f','%f','%f'], world estimate=['%f','%f','%f']",
                 current_pos_.x, current_pos_.y, current_yaw_rad_,world_est_current_pos_x_,world_est_current_pos_y_, world_est_current_yaw_rad_ );
    this->prev_pos_x = current_pos_.x;
    this->prev_pos_y = current_pos_.y;
    this->prev_world_est_current_yaw_rad_ = world_est_current_yaw_rad_;


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
  double world_est_current_pos_x_= 0;
  double world_est_current_pos_y_= 0;
  double prev_pos_x, prev_pos_y, prev_world_est_current_yaw_rad_;
  double offset_yawrad = 0;
  bool is_first_odom_callback = true;
  //--------  Kinematic related private variables --------// 
  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto understand_odom_node = std::make_shared<Odometry>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(understand_odom_node);
  executor.spin();
  /* TODO 
  - Should I normalize the pid_error_rotation theta_error = thetag - theta_pos?
  */

  //rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}