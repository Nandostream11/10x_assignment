# Custom DWA Local Planner in ROS2 Humble with Turtlebot3

Steps to run the assignment:-
1. Setup the workspace
```
mkdir 10x_av_ws
cd 10x_av_ws
git clone https://github.com/Nandostream11/10x_assignment.git ./src
```
2. Build the workspace
```
cd ~/10x_av_ws
colcon build
```
3. Launch Turtlebot3 in a world
4. Run the Custom DWA script

If you only want to access the topic outputs instead of the simulation, run the below pre-recorded rosbag:-
```
cd ~/10x_av_ws
ros2 bag play 10x_assg_bag 
```

