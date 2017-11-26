# RRT Path Planner Plugin Dependency Installation Instructions
This file has installation instructions for packages that the RRT path planner plugin relies on.

- [ROS Kinetic Kame](#ros)
- [Catkin](#catkin)

## ROS
Full instructions for installing ROS Kinetic Kame can be found [here](http://wiki.ros.org/kinetic/Installation).

The following instructions are for an Ubuntu installation.

1. Set up your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Set up your keys
```http://wiki.ros.org/gtest
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

3. Installation
Make sure your Debian package is up to date:
```
sudo apt-get update
```

Perform a full desktop install:
```
sudo-apt-get install ros-kinetic-desktop-full
```

4. Initialize rosdep
```
sudo rosdep init
rosdep update
```

5. Environment set up
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6. Install rosinstall
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Catkin
Full instructions for installing Catkin can be found [here](www.ros.org/wiki/catkin#Installing_catkin).

```
sudo apt-get install ros-lunar-catkin
```

## RViz
Full instructions for installing Rviz can be found [here](http://wiki.ros.org/rviz/UserGuide#Install_or_build_rviz)

```
sudo apt-get install ros-kinetic-rviz
```
