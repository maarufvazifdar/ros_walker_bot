/**
 * MIT License
 *
 * Copyright (c) 2021 Maaruf Vazifdar
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file move_bot.hpp
 * @author Maaruf Vazifdar
 * @brief Header file for Class MoveBot, its attributes and members.
 * @version 1.0
 * @date 11/23/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

#pragma once
class MoveBot {
 public:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  geometry_msgs::Twist vel;

  /**
   * @brief Constructor for class MoveBot
   * @param void
   */
  MoveBot() {
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);    
    sub = n.subscribe("scan", 10, &MoveBot::LaserscanCallback, this);
  };

  /**
   * @brief Commands turtlebot to turn left.
   * @param void 
   * @return void
   */  
  void turnLeft();

  /**
   * @brief Commands turtlebot to turn right. 
   * @param void 
   * @return void
   */
  void  turnRight();

  /**
   * @brief Commands turtlebot to move forward. 
   * @param void 
   * @return void
   */
  void moveForward();

  /**
   * @brief Callback for topic "scan" subscriber. 
   * @param const sensor_msgs::LaserScan::ConstPtr& msg 
   * @return void
   */
  void LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};
