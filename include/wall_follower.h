#ifndef wall_follower_H
#define wall_follower_H

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include "std_srvs/Empty.h"
#include <ctime>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <string>

#define TARGET_DISTANCE 0.20

class WallFollower {

private:
  // Node handle
  ros::NodeHandle n;

  // Publishers
  ros::Publisher cmd_vel_pub;

  // Subscribers
  ros::Subscriber front_ir_sub;
  ros::Subscriber left_ir_sub;
  ros::Subscriber right_ir_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber key_sub;

  // External Parameters
  bool left;
  bool right;

  // Global variables
  float front_distance;
  float left_distance;
  float right_distance;

  // PID control
  float old_prop_error;
  float integral_error;
  float KP = 10;
  float KI = 0.0;
  float KD = 0.0;
  float time_interval = 0.1;

  // Helper variables
  bool robot_lost;
  int lost_counter;
  bool robot_stop;
  float robot_x, robot_y;

  geometry_msgs::Twist calculateCommand();
  void frontIRCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  void leftIRCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  void rightIRCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  void keyCallback(std_msgs::Int16 key_msg);
  float calculateGain(float value);
  void calculateRobotLost();

  void saveMap();
  void saveRobotPose();
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

public:
  WallFollower();
  void run();
};

#endif