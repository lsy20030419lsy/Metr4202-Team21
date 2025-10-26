# Metr4202-Team21
Download all the files from github:
cd ~/metr4202_ws/src
git clone https://github.com/lsy20030419lsy/Metr4202-Team21.git

colcon build
source install/setup.bash

Open the test map made by our team (file name depends on the name on your labtop, in my world launcher/launch): 
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch my_world_launcher testworld1.launch.py

Open slam and rviz (remember when you doing the simulation, make use_sim_time:=True, if you using real robot, make use_sim_time:=Fales):
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

Open waypoint:
source install/setup.bash 
ros2 run waypoint_commander_team21 waypoint_cycler_team21

Open aruco detector:
ros2 run aruco_detector_team21 aruco_detector_team21



