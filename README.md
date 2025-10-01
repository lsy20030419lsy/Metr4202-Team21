# Metr4202-Team21

Copy and Paste the following code depending on what you need in the terminal:

DOWNLOAD WAYPOINT_COMMANDER FOLDER

cd ~/[ENTER YOUR WORKSPACE NAME HERE]/src
git init waypoint_commander
cd waypoint_commander
git remote add origin https://github.com/lsy20030419lsy/Metr4202-Team21.git
git config core.sparseCheckout true
echo "waypoint_commander/" >> .git/info/sparse-checkout
git pull origin main

UPDATE WAYPOINT_COMMANDER FOLDER

cd ~/[ENTER YOUR WORKSPACE NAME HERE]/src/waypoint_commander
git pull origin main

REBUILD AND SOURCE AFTER DOWNLOAD

cd ~/[ENTER YOUR WORKSPACE NAME HERE]
colcon build     
source install/setup.bash  

TO RUN THE PROGRAM 

1.OPEN GAZEBO IN A NEW TERMINAL
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2.OPEN SLAM IN A NEW TERMINAL
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True


3.OPEN RVIZ IN A NEW TERMINAL
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
map:=/path/to/turtlebot3_world_map.yaml

4. RUN THE PROGRAM
ros2 run waypoint_commander waypoint_cycler


