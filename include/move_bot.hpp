#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

#pragma once
class MoveBot {
 public:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  geometry_msgs::Twist vel;

  MoveBot() {
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);    
    sub = n.subscribe("scan", 10, &MoveBot::LaserscanCallback, this);
  };

  void turnLeft();

  void  turnRight();

  void moveForward();

  void LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};
