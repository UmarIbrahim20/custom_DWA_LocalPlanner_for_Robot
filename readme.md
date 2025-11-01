

mkdri custum_dwa
mkdir src

cd custom_dwa/src
git clone repolink

cd ~/custom_dwa
colcon build
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_bringup rviz2.launch.py

ros2 run custom_dwa_local_planner custom_dwa_node 