# Metr4202-Team21

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


