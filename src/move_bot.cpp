#include <move_bot.hpp>
#include <ros/ros.h>

  
void MoveBot::turnLeft() {
  ros::Rate rate(10);
    vel.linear.x = 0;
    vel.angular.z = 0.3;
    ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z);
    pub.publish(vel);
    ROS_INFO_STREAM("Turning Left");
}

void MoveBot::turnRight() {
  ros::Rate rate(10);
    vel.linear.x = 0;
    vel.angular.z = -0.3;
    ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z);
    pub.publish(vel);
    ROS_INFO_STREAM("Turning Right");
}

void MoveBot::moveForward() {
  ros::Rate rate(10);
  vel.angular.z = 0;
  vel.linear.x = 0.2;
  ROS_DEBUG_STREAM("Forward velocity= " << vel.linear.x << ", Turn velocity=" << vel.angular.z);
  pub.publish(vel);
  ROS_INFO_STREAM("Moving Forward");  
}

void MoveBot::LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(msg->ranges[330] < 0.3 || msg->ranges[0] < 0.3)
    turnLeft();
  else if(msg->ranges[30] < 0.3)
    turnRight();
  else
    moveForward();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");

  MoveBot M1;
  ros::spin();

  return 0;
}
