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

1.OPEN GAZEBO IN A NEW TERMINAL：

export TURTLEBOT3_MODEL=waffle_pi


ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2.OPEN SLAM IN A NEW TERMINAL：

ros2 launch slam_toolbox online_async_launch.py

3.OPEN RVIZ IN A NEW TERMINAL：

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
map:=/path/to/turtlebot3_world_map.yaml

4. RUN THE PROGRAM：

ros2 run waypoint_commander waypoint_cycler


5. RUN COMMAND WITHIN THE WORKSPACE:
   
ros2 run aruco_detector aruco_detector





🧩 方案一：从 GitHub 上只下载这三个文件夹（最简单）

你可以使用 Git Sparse Checkout（稀疏检出）功能，
只克隆仓库中指定的几个包。

✅ 步骤：

1️⃣ 在目标电脑上选择保存路径：

mkdir -p ~/metr4202_ws/src
cd ~/metr4202_ws/src


2️⃣ 初始化仓库：

git init
git remote add origin https://github.com/lsy20030419lsy/Metr4202-Team21.git
git config core.sparseCheckout true


3️⃣ 指定只下载的包（写入要保留的路径）：

echo "aruco_detector_team21/" >> .git/info/sparse-checkout
echo "waypoint_commander_team21/" >> .git/info/sparse-checkout
echo "my_world_launcher/" >> .git/info/sparse-checkout


4️⃣ 拉取仓库：

git pull origin main


💡 现在你在 ~/metr4202_ws/src 下只会看到：

aruco_detector_team21/
waypoint_commander_team21/
my_world_launcher/





