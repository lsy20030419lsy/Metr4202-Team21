# Metr4202-Team21
1. Download all project files from github:
cd ~/metr4202_ws/src
git clone https://github.com/lsy20030419lsy/Metr4202-Team21.git

2. Then build the workspace and source the setup file:
colcon build
source install/setup.bash

3. For simulation: Launch the test map by (File name may vary depending on laptop setup and world file name in my_world_launcher/launch): 
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch my_world_launcher test_world.launch.py

4. Start slam and rviz (for simulation: use_sim_time:=True; for real robot: use_sim_time:=Fales):
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

5.Re-source the workspace before running custom nodes:
source install/setup.bash 

6.Run Waypoint Commander
ros2 run waypoint_commander_team21 waypoint_cycler_team21

7.Run AruCo detector:
ros2 run aruco_detector_team21 aruco_detector_team21

8.



