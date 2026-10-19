cd ~/catkin_ws/src
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws
catkin_make
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
