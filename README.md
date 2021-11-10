# Installation of OpenRAVE and ROS
## Basic tools
```
# Python
sudo apt-get update
sudo apt-get install ipython python-dev python-numpy python-pip python-scipy
# check versions
python -c "import IPython; print('IPython v{}'.format(IPython.__version__))"
python -c "import numpy; print('numpy v{}'.format(numpy.__version__))"
python -c "import scipy; print('scipy v{}'.format(scipy.__version__))"

# git
sudo apt-get install git
git config --global user.name "your-github-username"
git config --global user.email "your-email@address.com"
```

## OpenRAVE
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
```
Test the installation with a built-in environment:
```
openrave data/lab1.env.xml
```

## OpenCV & Point Cloud Library (PCL)
```
sudo apt-get update
sudo apt-get install libopencv-dev python-opencv
sudo apt install libpcl-dev pcl-tools
```

## Gazebo
