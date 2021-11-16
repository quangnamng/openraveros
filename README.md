# Installation of OpenRAVE and ROS
## Basic tools
```
# Python
sudo apt-get update
sudo apt-get install ipython python-dev python-numpy python-pip python-scipy -y
# check versions
python -c "import IPython; print('IPython v{}'.format(IPython.__version__))"
python -c "import numpy; print('numpy v{}'.format(numpy.__version__))"
python -c "import scipy; print('scipy v{}'.format(scipy.__version__))"

# git
sudo apt-get install git
git config --global user.name "your-github-username"
git config --global user.email "your-email@address.com"

# other tools
sudo apt install curl nano gedit ssh vim mayavi2 -y
```


## ROS
Setup sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup keys
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Install ROS
```
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
```

Environment setup
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Dependencies for ROS packages
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools -y
```

The gazebo_ros_pkgs packages:
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control -y
```

Initialize rosdep
```
sudo apt install python-rosdep -y
sudo rosdep init
rosdep update
```

Setup catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make install
```


## OpenRAVE
Some dependencies need to be installed manually
```
sudo apt-get install blender openscad python-rtree -
pip install --upgrade pip # skip this if pip has errors in Ubuntu 16.04
pip install trimesh       # needed for working with OpenRAVE objects
# if fail, try this:
pip install control trimesh --user 
```

Clone the repository:
```
git clone https://github.com/crigroup/openrave-installation.git
```
Go to the directory just downloaded and run the scripts:
```
cd openrave-installation
./install-dependencies.sh
./install-osg.sh -j6
./install-fcl.sh -j6
./install-openrave.sh -j6
cd && rm -rf openrave-installation
```
Test the installation with a built-in environment:
```
openrave data/lab1.env.xml
```

## OpenCV & PCL
```
sudo apt-get update
sudo apt-get install libopencv-dev python-opencv -y
sudo apt install libpcl-dev pcl-tools -y
```


