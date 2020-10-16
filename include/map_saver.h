/**
 * @file map_saver.h
 * @author Erwin Lejeune (erwin.lejeune15@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef MAP_SAVER_H
#define MAP_SAVER_H

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

class MapSaver {

private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;

	float robot_x, robot_y;
	int loop_counter;

	void saveMap();
	void saveRobotPose();
	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

public:
    MapSaver();
    void run();
};

#endif
