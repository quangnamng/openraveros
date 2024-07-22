# Installation of ROS and OpenRAVE for Robotics
This is based on the [Open-source Robotics](https://osrobotics.org/osr/) course by [CRI Group](https://personal.ntu.edu.sg/cuong/).
* System requirements: Ubuntu 18.04 or Ubuntu 16.04 using Python 2 
(instruction for Ubuntu 20.04 + Python 3 will be in a separate branch)
* Please replace `melodic` (Ubuntu 18.04) with `kinetic` (Ubuntu 16.04) whenever necessary
* Please read the comments carefully at every step


## Basic tools
```
# Python 2
sudo apt-get update
sudo apt-get install ipython python-dev python-numpy python-pip python-scipy -y
# check versions
python -c "import IPython; print('IPython v{}'.format(IPython.__version__))"
python -c "import numpy; print('numpy v{}'.format(numpy.__version__))"
python -c "import scipy; print('scipy v{}'.format(scipy.__version__))"

# git
sudo apt-get install git -y
git config --global user.name "your-github-username"
git config --global user.email "your-email@address.com"

# other tools
sudo apt install curl nano gedit ssh vim -y
pip install --upgrade pip # skip this if pip causes errors in Ubuntu 16.04
pip install future        # missing compatibility layer between Python 2 and Python 3
```

In Ubuntu 18.04, it is safer to set the default Python version to Python 2:
```
# checking
python --version
python3 --version

# in the next commands, replace 'python2.7' and 'python3.5' by the versions you get previously
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 2 
sudo update-alternatives --config python   # type `1` to choose python2.7 
```


## Robot Operating System (ROS)
Setup `sources.list`
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup keys
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Install ROS: (for Ubuntu 16.04, replace `melodic` with `kinetic`)
```
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y
```

Environment setup
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Dependencies for ROS packages
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools -y
```

Install `gazebo_ros_pkgs`
```
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control -y
```

Install `ros_control`
```
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
```

Initialize `rosdep`
```
sudo rosdep init
rosdep update
```

Some dependencies need to be installed manually:
```
sudo add-apt-repository ppa:openscad/releases
sudo apt-get install blender openscad python-rtree -y
```


## OpenRAVE
Clone the repository
```
cd && git clone https://github.com/crigroup/openrave-installation.git
cd openrave-installation
```
!!! Run this line for Ubuntu 18 (ROS Melodic) only:
```
# In Ubuntu 18.04, use the next line to check out an old commit that installs OpenRAVE 0.9.0
# Because latest commit will install OpenRAVE 0.9.0 for Ubuntu 16.04 but 0.53.1 for Ubuntu 18.04
git checkout b2766bd789e2432c4485dff189e75cf328f243ec
```
Go to the directory just downloaded and run the scripts
```
# install using scripts
./install-dependencies.sh
./install-osg.sh
./install-fcl.sh
./install-openrave.sh
cd && sudo rm -rf openrave-installation
```
Test the installation with the built-in environment and/or some [examples](http://openrave.org/docs/latest_stable/examples/)
```
openrave data/lab1.env.xml
openrave.py --example hanoi
```

(Optional) Install trimesh (may be needed for working with OpenRAVE objects)
```
pip install control trimesh
# if the above fails, try:
pip install --no-deps control trimesh
```


## (Optional) OpenCV & PCL
```
sudo apt-get update
sudo apt-get install libopencv-dev python-opencv -y
sudo apt install libpcl-dev pcl-tools -y
```


## osr_course_pkgs (Ubuntu 16.04 ONLY)
The Open-source Robotics course page is [here](https://osrobotics.org/osr/).

Below is how to build the ROS package [osr_course_pkgs](https://github.com/crigroup/osr_course_pkgs.git) 
for that course in your own [catkin](https://wiki.ros.org/catkin/Tutorials) workspace:
* Make a `catkin_ws` directory to store the packages:
```
cd && mkdir -p ~/catkin_ws/src
```
* Clone the repository [osr_course_pkgs](https://github.com/crigroup/osr_course_pkgs.git):
```
cd ~/catkin_ws/src
git clone https://github.com/crigroup/osr_course_pkgs.git
```
* Prepare to build this package:
```
cd ~/catkin_ws/src
wstool init .
wstool merge osr_course_pkgs/dependencies.rosinstall
wstool update
rosdep update
rosdep install --from-paths . --ignore-src -y
```
* Build all packages inside `catkin_ws`:
```
cd ~/catkin_ws
catkin_make --install
```
* First time building `catkin_ws`? do this:
```
echo "source /home/`id -un`/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
* Lastly, after building new packages, run `source ~/.bashrc` to source the setup file.

Check the built packages: 
* Run an example in gazebo:
```
roslaunch osr_gazebo cubes_task.launch
```
* Escape by `Ctrl+C`
* Troubleshoot: if you do not see the table in front of the robot, it is because
gazebo models are not downloaded automatically, you may clone them to your computer:
(the size is large, afterwards, you may want to keep only the models you need)
```
cd .gazebo
git clone https://github.com/osrf/gazebo_models.git
mv gazebo_models/ models
```

Example: run the cubes task by the following commands in 3 different terminals in the following order:
* On terminal 1:
```
roslaunch osr_gazebo cubes_task.launch
```
* On terminal 2:
```
roslaunch osr_control controllers.launch
```
* On terminal 3:
```
rosrun osr_examples gazebo_pick_and_place.py
```
Notes:
* The first time you run this it may take a few minutes to generate robot's kinematics data.
* Escape OpenRAVE by typing `exit` into the terminal then pressing Enter, while escape other programs by `Ctrl+C`.


