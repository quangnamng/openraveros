# Installation of ROS and OpenRAVE for Robotics
This is based on the Open-source Robotics course which is available [here](https://osrobotics.org/osr/).

IMPORTANCE:
* Below is the instruction for Ubuntu 16.04 Xenial Xerus. 
* For Ubuntu 18.04 Bionic Beaver, replace `kinetic` by `melodic` when necessary unless otherwise stated. 
* This instruction may not work for Ubuntu 20.04 and above.
* Please read the comments at every step.

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
sudo apt-get install git -y
git config --global user.name "your-github-username"
git config --global user.email "your-email@address.com"

# other tools
sudo apt install curl nano gedit ssh vim -y
pip install --upgrade pip # skip this if pip causes errors in Ubuntu 16.04
pip install future        # missing compatibility layer between Python 2 and Python 3
```

In Ubuntu 18.04, it is safer to set the default Python version to Python 2 using the following commands:
```
# checking
python --version
python3 --version

# in the next commands, replace 'python2.7' and 'python3.5' by the versions you get from above
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 2 
sudo update-alternatives --config python   # type `1` to choose python2.7 
```


## ROS
Setup `sources.list`
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

Install `gazebo_ros_pkgs`
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control -y
```

Install `ros_control`
```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

Initialize `rosdep`
```
sudo rosdep init
rosdep update
```

Some dependencies need to be installed manually
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

Install trimesh (needed for working with OpenRAVE objects)
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



## osr_course_pkgs
The course page is [here](https://osrobotics.org/osr/).

We build `catkin_ws` using `catkin_tools`: 
* Install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) by `sudo apt install python-catkin-tools`
* Make a directory:
```
cd && mkdir -p ~/catkin_ws/src
```
* Clone the repository [osr_course_pkgs](https://github.com/crigroup/osr_course_pkgs.git):
```
cd ~/catkin_ws/src
git clone https://github.com/crigroup/osr_course_pkgs.git
```
* Initialize catkin workspace:
```
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/$ROSDISTRO
catkin config --merge-devel
```
* Prepare to build this package:
```
cd ~/catkin_ws/src
wstool init .
wstool merge osr_course_pkgs/dependencies.rosinstall
wstool update
sudo apt update
rosdep update --include-eol-distros
sudo rosdep install --rosdistro $ROSDISTRO --ignore-src --from-paths . -y
```
* Build all packages inside `catkin_ws`:
```
cd ~/catkin_ws
catkin config --install
catkin build
```
* First time building `catkin_ws`? do this:
```
echo "source /home/`id -un`/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
```
* Lastly, after building new packages, run `source ~/.bashrc` to source the setup.

Check the built packages: 
* Run an example in gazebo:
```
roslaunch osr_gazebo cubes_task.launch
```
* Escape by `Ctrl+C`
* Troubleshoot: if you do not see the table in front of the robot,
it is because gazebo models are not downloaded automatically, clone them to your computer:
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
* Escape OpenRAVE by typing `exit` into terminal 3, and escape others by `Ctrl+C`


