# project_server
Dependency
```bash
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-teleop-twist-joy
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-laser-proc
sudo apt-get install ros-kinetic-rgbd-launch
sudo apt-get install ros-kinetic-depthimage-to-laserscan
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial-python
sudo apt-get install ros-kinetic-rosserial-server
sudo apt-get install ros-kinetic-rosserial-client
sudo apt-get install ros-kinetic-rosserial-msgs
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-urdf
sudo apt-get install ros-kinetic-xacro
sudo apt-get install ros-kinetic-compressed-image-transport
sudo apt-get install ros-kinetic-rqt-image-view
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-interactive-markers
```
Installation
```bash
cd catkin_ws/src
git clone https://github.com/seungchul0406/project_server
cd ..
catkin_make
source ~/.bashrc && source ~/catkin_ws/devel/setup.bash
```

## Usage

### Running an environment
slam
```bash
roslaunch slam_navi slam.launch
```

navi
```bash
roslaunch slam_navi navi.launch
```

teleop
```bash
roslaunch teleop teleop_key.launch
```

autonomous slam
```bash
roslaunch auto_slam single_sim.launch
```