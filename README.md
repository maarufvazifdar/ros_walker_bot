# ros_walker_bot
Sumilation of turtlebot3 burger avoiding obstacles in a closed environment.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

### **Author:** *Maaruf Vazifdar*, maarufvazifdar@gmail.com

## Overview
The ROS-gazebo assignment demonstrating simple obstacle avoidance behaviour using a lidar sensor. When the turtlebot detects an obstacle within an certian specified range, it stops moving, turns left or right till the obstacle is avoided and proceeds moving forward.

The walker node subscribes to the *scan* topic to get the laser data, and based on whether obstacle is detected published velocity on the *cmd_vel* topic to move the robot forward, turn left or right.

## Dependencies
- ROS - Melodic
- Gazebo 9+
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)


## Building and Running
1) Build the package: 
   ```bash
    sudo apt install ros-melodic-turtlebot3-simulations
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

    cd ~/<your_ws>/src
    git clone https://github.com/maarufvazifdar/ros_walker_bot.git
    cd ~/<your_ws>
    catkin_make
    ```

2) To launch turtlebot in gazebo world, walker node for obstacle avoidance and RViz to visualize Robot Model and sensor data:
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    roslaunch ros_walker_bot ros_walker_bot.launch
    ```
    To record ros bag file give argument record_rosbag as *true*.
    ```bash
    roslaunch ros_walker_bot ros_walker_bot.launch record_rosbag:=true
    ```
    You sholud be able to see Turtlebot moving in the gazebo world, avoiding obstacles, and also visualize the Robot Model and laser data in RViz.

3) Once you have recorded ros bag files, to inspect and playback data:
    ```bash
    roscore
    ```
    In a new terminal:
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosrun rviz rviz -d src/ros_walker_bot/rviz/ros_walker_bot_rosbag.rviz
    ```
    In a new terminal:
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosbag info src/ros_walker_bot/results/ros_walker_bot_bagfile.bag
    rosbag play src/ros_walker_bot/results/ros_walker_bot_bagfile.bag 
    ```
    You sholud be able to visualize the Robot TF and laser data in RViz. 

## Run cpplint and cppcheck
To run Cpplint:
  ```bash
  cpplint $( find . -name \*.hpp -or -name \*.cpp) > results/cpplint_result.txt
  ```

To run Cppcheck:
  ```bash
  cppcheck --language=c++ --std=c++11 -I include --suppress=missingIncludeSystem  $( find . -name \*.hpp -or -name \*.cpp) > results/cppcheck_result.txt
  ```

## License
MIT License
```
Copyright (c) 2021  Mohammed Maaruf Vazifdar.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```
