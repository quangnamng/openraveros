# Installation of ROS and OpenRAVE
This is the instruction for Ubuntu 16.04 Xenial Xerus. For Ubuntu 18.04 Bionic Beaver, replace 'kinetic' by 'melodic' unless otherwise stated.

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
pip install --upgrade pip # skip this if pip causes errors in Ubuntu 16.04
pip install future        # missing compatibility layer between Python 2 and Python 3
```

In Ubuntu 18.04 or later, it is safer to set the default Python version to Python 2 using the following commands:
```
sudo update-alternatives --config python                                       # check whether is was set up before
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2 # this may be 'python3.5', check by 'python3 --version'
sudo update-alternatives --config python                                       # type `1` to choose python2.7 
```

If using WSL, install VS Code: 
- In Windows, download VS Code and install some extensions: Remote-WSL, Python, C++, Docker
- In WSL distro, to open the current work directory in VS Code, just run:
```
code .
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

Initialize catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```

Some dependencies need to be installed manually
```
sudo add-apt-repository ppa:openscad/releases
sudo apt-get install blender openscad python-rtree -
```


## OpenRAVE
Clone the repository:
```
cd && git clone https://github.com/crigroup/openrave-installation.git
```
Go to the directory just downloaded and run the scripts:
```
cd openrave-installation
# the tag `-j4` allows using 4 CPU cores to compile, lower this number if c++ compiler fails
./install-dependencies.sh -j4 # it may take a while at the end of the process, do not interrupt
./install-osg.sh -j4          # this may require user's password before building
./install-fcl.sh -j4
./install-openrave.sh -j4
cd && sudo rm -rf openrave-installation
```
Test the installation with the built-in environment and/or some [examples](http://openrave.org/docs/latest_stable/examples/):
```
openrave data/lab1.env.xml
openrave.py --example hanoi
```

Install trimesh (needed for working with OpenRAVE objects)
```
pip install control trimesh # needed for working with OpenRAVE objects
# if fail, try this:
pip install --no-deps control trimesh
```


## (Optional) OpenCV & PCL
```
sudo apt-get update
sudo apt-get install libopencv-dev python-opencv -y
sudo apt install libpcl-dev pcl-tools -y
```
