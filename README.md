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
    ros2 launch turtlebot3_gazebo world.launch.py
    ```
4. Run the Custom DWA script
    ```bash
    source ~/10x_av_ws/install/setup.bash
    ros2 run dwa_custom_planner nav_goal_dwa
    ```
5. Call the service to pass goal coordinates[eg: (1.2,2.0)]
    ```bash
    source ~/10x_av_ws/install/setup.bash
    ros2 service call /togoal dwa_custom-planner/srv/ToGoal "{goal_x: 1.2, goal_y: 2.0}"
    ```

File tree:-
.
├── dwa_custom_planner
│   ├── CMakeLists.txt
│   ├── include
│   │   └── dwa_custom_planner
│   │       └── dwa_planner_custom.hpp
│   ├── package.xml
│   ├── src
│   │   └── nav_goal_dwa.cpp
│   └── srv
│       └── ToGoal.srv
└── README.md

The turtlebot3_gazebo package is also added to the src to simulate the Trutlebot3 

Main algorithm Workflow-
[nav_goal_dwa]: Receive goal(x,y) from service -> Send goal iteratively to dwa_planner_custom -> Publish sampled trajectories, best trajectory marker array(to dwa_trajectories) -> Publish the best velcoity pairs

[dwa_planner_custom]: obstacles map(scan) -> Velosity (Linear, Angular) sampling -> iteration over cost function (minimization) -> Local Planning

Below is the video demonstration:
https://youtu.be/y8iu5mr0jKo