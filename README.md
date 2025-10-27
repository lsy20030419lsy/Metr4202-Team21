# Metr4202-Team21
1. Download all project files from github:
cd ~/metr4202_ws/src
git clone https://github.com/lsy20030419lsy/Metr4202-Team21.git

2. Then build the workspace and source the setup file:
colcon build
source install/setup.bash

3. *For simulation: Launch the test map by (File name may vary depending on laptop setup and world file name in my_world_launcher/launch): 
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch my_world_launcher test_world.launch.py

4. (Open a new terminal) Start slam and rviz (for simulation: use_sim_time:=True; for real robot: use_sim_time:=Fales):
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

5. (Open a new terminal) Run AruCo detector:
source install/setup.bash 
ros2 run aruco_detector_team21 aruco_detector_team21

6. (Open a new terminal) Run Waypoint Commander
source install/setup.bash 
ros2 run waypoint_commander_team21 waypoint_cycler_team21

7. *For real robot:
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID={XX}
or put in bashrc by (nano ~/.bashrc and source ~/.bashrc)

8. Make sure router is connected to the device and open a new terminal to connect the robot by:
ssh ubuntu@192.168.8.id
ros2 launch turtlebot3_bringup robot.launch.py

9. (Open a New Terminal) Set up the calibration file
    Move the saved camera calibration zip file to home directory:
mv ~/metr4202_ws/src/Metr4202-Team21/calibrationdata.tar.gz ~/
    Extract the files from the folder:
tar -xzvf calibrationdata.tar.gz
    Locate the YAML file, e.g.  camera.yaml:
ls
     Copy the file into camera_info folder using scp:
scp <camera_name>.yaml ubuntu@192.168.8.<id>:/home/ubuntu/.ros/camera_info/
    Re-run the camera node on the turtlebot to use the new calibration data:
ros2 run v4l2_camera v4l2_camera_node
    Verify that the camera calibration file is being used. It should show something in the logs if it is working:
ros2 topic echo /camera_info | head -n 30

10. (Open a New Terminal) To run the slam:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=True

11. (Open a New Terminal) To run aruco_detector, first open camera node:
ssh ubuntu@192.168.8.xx
ros2 run v4l2_camera v4l2_camera_node

12. (Open a New Terminal) To open the camera:
export ROS_DOMAIN_ID=36
ros2 run rqt_image_view rqt_image_view

13. (Open a New Termina) To run the aruco_dector:
source install/setup.bash 
ros2 run aruco_detector_team21 aruco_detector_team21

14. (Open a New Terminal) To run waypoint_commendar:
source install/setup.bash
ros2 run waypoint_commander_team21 waypoint_cycler_team21




