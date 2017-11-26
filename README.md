# Randomly Exploring Random Tree Path Planner ROS Plugin
This plugin implements a simulated Randomly Exploring Random Tree (RRT) path planner for the Kinetic Kame release of the Robot Operating System (ROS) framework. It uses the nav_core::BaseGlobalPlanner interface and can be used on any platforms that implement the move_base package, but has been specifically tested using the simulated Turtlebot platform.

## Table of Contents
- [Personnel](#personnel)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)

## Personnel
This plugin was created by [Jessica Howard](jmhoward@umd.edu) as a final project for the University of Maryland course ENPM808X - Software Development for Robotics during the Fall 2017 semester.

## Installation
Clone the package into your catkin workspace:
```
cd [workspace]/src
git clone https://github.com/jeshoward/turtlebot_rrt.git
cd ..
catkin_make
```

### Requirements
1. ROS Kinetic Kame [(Installation Instructions)](docs/README_DEPENDENCIES.md#ros)
2. Catkin [(Installation Instructions)](docs/README_DEPENDENCIES.md#catkin)
3. Rviz [(Installation Instructions)](/docs/README_DEPENDENCIES.md#rviz)

## Usage

## License
BSD 3-Clause License

Copyright (c) 2017, Jessica Howard
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
