# Installation of ROS and OpenRAVE for Robotics
Below is the instruction for Ubuntu 16.04 Xenial Xerus. 
For Ubuntu 18.04 Bionic Beaver, replace `kinetic` by `melodic` when necessary unless otherwise stated. 
This instruction may not work for Ubuntu 20.04 and above.


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
Go to the directory just downloaded and run the scripts
```
# In Ubuntu 18.04, use the next line to check out an old commit that installs OpenRAVE 0.9.0
# Because latest commit will install OpenRAVE 0.9.0 for Ubuntu 16.04 but 0.53.1 for Ubuntu 18.04
git checkout b2766bd789e2432c4485dff189e75cf328f243ec

# install using scripts
./install-dependencies.sh -j4
./install-osg.sh -j4
./install-fcl.sh -j4
./install-openrave.sh -j4
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


# Maintainer
* [Quang Nam Nguyen](mailto:quangnam.nguyen@ntu.edu.sg)
