# checkpoint17_pid_maze_solver

## best value so far
    case Simulation:
        this->Kp = 1.4;
        this->Ki = 0.05;
        this->Kd = 0;
        this->Kp_angle = 1.2;
        this->Ki_angle = 0.005; 
        this->Kd_angle = 0.005; 
        RCLCPP_INFO(this->get_logger(), "Simulation Scene,Kp:%f,Ki:%f,Kd:%f,Kp_angle:%f,Ki_angle:%f,Kd_angle:%f",
       this->Kp, this->Ki, this->Kd, this->Kp_angle, this->Ki_angle, this->Kd_angle);   
        if(!this->is_forward_direction_){
            std::reverse(this->ref_points_simulation.begin(), this->ref_points_simulation.end());
        }

    case Cyberworld:
        this->Kp = 0.5;
        this->Ki = 0;
        this->Kd = 0;
        this->Kp_angle = 0.5;
        this->Ki_angle = 0;
        this->Kd_angle = 0;
        RCLCPP_INFO(this->get_logger(), "Cyberworld Scene,Kp:%f,Ki:%f,Kd:%f,Kp_angle:%f,Ki_angle:%f,Kd_angle:%f",
       this->Kp, this->Ki, this->Kd, this->Kp_angle, this->Ki_angle, this->Kd_angle);  
        if(!this->is_forward_direction_){
            std::reverse(this->ref_points_cyberworld.begin(), this->ref_points_cyberworld.end());
        }

## to repeat the last success
Terminal 1
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py

Terminal 2
cd ~/ros2_ws && colcon build 
source install/setup.bash ; ros2 run pid_maze_solver pid_maze_solver 2.1 0.001 0.1 1 0 0


ros2 topic echo /odometry/filtered --field pose.pose.position