# checkpoint17_pid_maze_solver

## best value so far
    Kp = 2.1;
    Ki = 0.001;
    Kd = 0.1;
    Kp_angle = 1;
    Ki_angle = 0; 
    Kd_angle = 0; 

## to repeat the last success
Terminal 1
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py

Terminal 2
cd ~/ros2_ws && colcon build 
source install/setup.bash ; ros2 run pid_maze_solver pid_maze_solver 2.1 0.001 0.1 1 0 0


ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 topic echo /rosbot_xl_base_controller/odom --field pose.pose.position