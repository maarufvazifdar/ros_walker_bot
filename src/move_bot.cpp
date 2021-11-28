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
 * @file move_bot.cpp
 * @author Maaruf Vazifdar
 * @brief brief.
 * @version 1.0
 * @date 11/23/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include <move_bot.hpp>
#include <ros/ros.h>


/**
 * @brief Publish positive angular velocity on cmd_vel topic to turn robot in 
 *        left direction.
 * @param void
 * @return void
 */
void MoveBot::turnLeft() {
  ros::Rate rate(10);
    vel.linear.x = 0;
    vel.angular.z = 0.3;
    ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z)
    pub.publish(vel);
    ROS_INFO_STREAM("Turning Left");
}

/**
 * @brief Publish negative angular velocity on cmd_vel topic to turn robot in 
 *        right direction.
 * @param void
 * @return void
 */
void MoveBot::turnRight() {
  ros::Rate rate(10);
    vel.linear.x = 0;
    vel.angular.z = -0.3;
    ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z)
    pub.publish(vel);
    ROS_INFO_STREAM("Turning Right");
}

/**
 * @brief Publish  positive linear velocity on cmd_vel topic to move robot in 
 *        forward direction.
 * @param void
 * @return void
 */
void MoveBot::moveForward() {
  ros::Rate rate(10);
  vel.angular.z = 0;
  vel.linear.x = 0.2;
  ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z)
  pub.publish(vel);
  ROS_INFO_STREAM("Moving Forward");  
}

/**
 * @brief Subscriber Callback for Lidar sensor.
 * @param sensor_msgs::LaserScan::ConstPtr& msg - laser data 
 * @return void
 */
void MoveBot::LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(msg->ranges[330] < 0.3 || msg->ranges[0] < 0.3)
    turnLeft();
  else if(msg->ranges[30] < 0.3)
    turnRight();
  else
    moveForward();
}


int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "walker");

  // Object of class MoveBot to call constructor and start moving the robot
  MoveBot M1;

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}
