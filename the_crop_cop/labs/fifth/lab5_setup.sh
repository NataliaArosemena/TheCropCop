dir=~/catkin_ws/src
if [ -d "$dir" ]; then
    echo "Directory $dir already exists."
else
    echo "Directory $dir does not exist. Creating now..."
    mkdir -p "$dir"
    cd "$dir"
    cd ..
    catkin_make
    source devel/setup.bash
    source ~/.bashrc
fi
sudo apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-server ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-navigation ros-noetic-interactive-markers ros-noetic-tf2-sensor-msgs git
rosdep update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/openslam_gmapping 
git clone https://github.com/ros-perception/slam_gmapping 
git clone https://github.com/ros-planning/navigation
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3.git
rosdep install --from-paths src/ -i
cd ~/catkin_ws
catkin_make
catkin_make install
source ~/.bashrc
exit