# Custom DWA Local Planner in ROS2 Humble with Turtlebot3

Steps to run the assignment:-
Installations:
- Ensure ROS2 Humble is installed and setup
- Gazebo Classic is installed
- Set waffle as defauft in export
    ```bash
    sudo apt install -y ros-humble-turtlebot3* 
    export TURTLEBOT3_MODEL=waffle
    ```
1. Setup the workspace
    ```bash
    mkdir 10x_av_ws
    cd 10x_av_ws
    git clone https://github.com/Nandostream11/10x_assignment.git ./src
    ```
2. Install dependencies and Build the workspace
    ```bash
    cd ~/10x_av_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    ```
3. Launch Turtlebot3 in a world
    ```bash
    source ~/10x_av_ws/install/setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
    ```
4. Run the Custom DWA script
    ```bash
    source ~/10x_av_ws/install/setup.bash
    ros2 run dwa_custom_planner nav_goal_dwa
    ```
5. Call the service to pass goal coordinates[eg: (1.2,2.0)]
    ```bash
    source ~/10x_av_ws/install/setup.bash
    ros2 service call /togoal dwa_custom_planner/srv/ToGoal "{goal_x: 1.2, goal_y: 2.0}"
    ```
6. In another terminal, run rviz2 for visualization of trajectories in DWA
   ```bash
   cd ~/10x_av_ws
   ros2 run rviz2 rviz2 -d src/10x.rviz
   ```

File tree:-                                \
.                                          \
├── dwa_custom_planner                     \
│   ├── CMakeLists.txt                     \
│   ├── package.xml                        \
│   ├── include                            \
│   │   └── dwa_custom_planner             \
│   │       └── dwa_planner_custom.hpp     \
│   ├── src                                \
│   │   └── nav_goal_dwa.cpp               \
│   └── srv                                \
│       └── ToGoal.srv                     \
└── README.md                              \

The turtlebot3_gazebo package is also added to the src to simulate the Trutlebot3 

Main algorithm Workflow-    \
[nav_goal_dwa]: Receive goal(x,y) from service -> Send goal iteratively to dwa_planner_custom -> Publish sampled trajectories, best trajectory marker array(to dwa_trajectories) -> Publish the best velcoity pairs

[dwa_planner_custom]: obstacles map(scan) -> Velosity (Linear, Angular) sampling -> iteration over cost function (minimization) -> Local Planning 

Custom DWA Algorithm approach:    \
1. Sampling all possible velocity pairs (v, w) within a feasible dynamic window in discrete step velocities(v_res, w_res) based on current speed and acceleration limits.
2. Predict robot motion over a short horizon (del_T = 2 s) with step dt_ = 0.05 s
3. Scores each trajectory using a cost function().
4. Selects the one with the lowest total cost.

Below is the video demonstration:           
<!-- <img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/2593add8-0d0b-4c60-92ba-157641a84c94" /> -->
[![Youtube Video](https://github.com/user-attachments/assets/2593add8-0d0b-4c60-92ba-157641a84c94)](https://youtu.be/y8iu5mr0jKo)

Improvements:
- Transfer all parameters to config/params.yaml file for abstraction & easing tuning
- Address edge cases
    - goal inside obstacle-> diminishing motion closest to goal
    - Auto detect bot flipping in unseen cases when control is withdrawn at non-zero cmd_vel-> IMU sensor to detect collision or illegal pose
- remove spin_some(dwa) -> Convert the service to action server to iteratively use dwa_planner_custom with feedbacks until goal is reached
