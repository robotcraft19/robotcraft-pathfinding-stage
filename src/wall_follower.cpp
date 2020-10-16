/**
 * @file wall_follower.cpp.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2019-08-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "wall_follower.h"

geometry_msgs::Twist WallFollower::calculateCommand() {
  // Create message
  auto msg = geometry_msgs::Twist();

  if (!this->robot_stop) {
    // Check if robot is lost (after 75 loops without sensing any wall)
    this->calculateRobotLost();

    if (right) {
      if (front_distance < TARGET_DISTANCE) {
        // Prevent robot from crashing
        msg.angular.z = 1.25; // maximum angular speed
        msg.linear.x = -0.04;
      } else if (robot_lost == true) {
        // Robot is lost, go straight to find wall
        msg.linear.x = 0.08;
      } else {
        // Robot keeps using normal PID controller
        float gain = calculateGain(right_distance);
        msg.linear.x = 0.08;
        msg.angular.z = gain;
      }
    }

    else if (left) {
      if (front_distance < TARGET_DISTANCE) {
        // Prevent robot from crashing
        msg.angular.z = -1.25; // maximum angular speed
        msg.linear.x = -0.04;
      } else if (robot_lost == true) {
        // Robot is lost, go straight to find wall
        msg.linear.x = 0.08;
      } else {
        // Robot keeps using normal PID controller
        float gain = calculateGain(left_distance);
        msg.linear.x = 0.08;
        msg.angular.z = gain;
      }
    }
  }

  else {
    // Stop robot (set velocities to 0)
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
  }

  return msg;
}

void WallFollower::frontIRCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Extract range, first (and only) element of array
  this->front_distance = msg->ranges[0];
}
void WallFollower::leftIRCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Extract range, first (and only) element of array
  this->left_distance = msg->ranges[0];
}
void WallFollower::rightIRCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Extract range, first (and only) element of array
  this->right_distance = msg->ranges[0];
}

void WallFollower::keyCallback(std_msgs::Int16 key_msg) {
  ROS_INFO("key typed: %d", key_msg.data);
  if (key_msg.data == int('s')) {
    ROS_INFO("Saving map, stopping robot");
    this->robot_stop = true;
    this->saveMap();
    this->saveRobotPose();
    ROS_INFO("Starting robot again...");
    this->robot_stop = false;
  }
  if (key_msg.data == int('x')) {
    ros::shutdown();
  }
}

float WallFollower::calculateGain(float value) {
  // Calculate errors
  float error = TARGET_DISTANCE - value;
  float new_der_err = error - this->old_prop_error;
  float new_int_err = this->integral_error + error;

  // Calculate gain
  float gain = this->KP * error + this->KI * new_int_err * this->time_interval +
               this->KD * new_der_err / this->time_interval;

  // Update old errors
  this->old_prop_error = error;
  this->integral_error = new_int_err;

  // Restrict gain to prevent overshooting on sharp corners
  if (left)
    gain = -gain;
  if (gain > 0.4)
    gain = 0.4;

  if (right)
    if (gain < -0.4)
      gain = -0.4;

  return gain;
}

void WallFollower::calculateRobotLost() {
  if (right) {
    // Calculations needed to check if robot is lost
    if (front_distance > TARGET_DISTANCE && right_distance > TARGET_DISTANCE &&
        left_distance > TARGET_DISTANCE) {
      ++lost_counter;

      // π / 0.4 ≈ 8.0, after 80 loops robot has made at least half a rotation
      if (lost_counter >= 100) {
        robot_lost = true;
        ROS_WARN("ROBOT LOST! SEARCHING WALL...");
      }
    } else if (front_distance < TARGET_DISTANCE ||
               right_distance < TARGET_DISTANCE) {
      robot_lost = false;
      lost_counter = 0;
    }
  } else if (left) {
    // Calculations needed to check if robot is lost
    if (front_distance > TARGET_DISTANCE && right_distance > TARGET_DISTANCE &&
        left_distance > TARGET_DISTANCE) {
      ++lost_counter;

      // π / 0.4 ≈ 8.0, after 80 loops robot has made at least half a rotation
      if (lost_counter >= 100) {
        robot_lost = true;
        ROS_WARN("ROBOT LOST! SEARCHING WALL...");
      }
    } else if (front_distance < TARGET_DISTANCE ||
               left_distance < TARGET_DISTANCE) {
      robot_lost = false;
      lost_counter = 0;
      // ROS_WARN("WALL FOUND");
    }
  }
}

void WallFollower::saveMap() {
  /* Runs map_saver node in map_server package to save
   *  occupancy grid from /map topic as image */

  // Save as map.pgm
  system("cd ~/catkin_ws/src/robotcraft-pathfinding-stage/scans && rosrun "
         "map_server map_saver -f map");
}

void WallFollower::saveRobotPose() {
  /* Saves the robot's latest pose which can be used
   *  for target position calculation */
  const char *homeDir = getenv("HOME");
  std::string file(homeDir);
  file.append(
      "/catkin_ws/src/robotcraft-pathfinding-stage/scans/robot_position.txt");

  std::ofstream position_file;
  position_file.open(file);
  position_file << this->robot_x << "\n" << this->robot_y << "\n";
  position_file.close();
}

void WallFollower::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  this->robot_x = msg->pose.pose.position.x;
  this->robot_y = msg->pose.pose.position.y;
}

WallFollower::WallFollower() {
  // Initialize ROS
  this->n = ros::NodeHandle();
  srand(time(NULL));

  n.getParam("left", this->left);
  n.getParam("right", this->right);

  int rnd = rand() % 100;

  if (left && right) {
    if (rnd > 50)
      left = false;
    else
      right = false;
  }

  if (!left && !right) {
    if (rnd > 50)
      left = true;
    else
      right = true;
  }

  ROS_INFO("Right = %d\n", right);
  ROS_INFO("Left = %d\n", left);

  // Setup publishers
  this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Setup subscribers
  this->front_ir_sub = this->n.subscribe("base_scan_1", 10,
                                         &WallFollower::frontIRCallback, this);
  this->left_ir_sub =
      this->n.subscribe("base_scan_2", 10, &WallFollower::leftIRCallback, this);
  this->right_ir_sub = this->n.subscribe("base_scan_3", 10,
                                         &WallFollower::rightIRCallback, this);
  this->odom_sub =
      this->n.subscribe("odom", 5, &WallFollower::odomCallback, this);
  this->key_sub =
      this->n.subscribe("/key_typed", 1, &WallFollower::keyCallback, this);
}

void WallFollower::run() {
  // Send messages in a loop
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    // Calculate the command to apply
    auto msg = calculateCommand();

    // Publish the new command
    this->cmd_vel_pub.publish(msg);

    // Receive messages and publish inside callbacks
    ros::spinOnce();

    // And throttle the loop
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "maze_basic_solver");

  // Create our controller object and run it
  auto controller = WallFollower();
  sleep(5);
  controller.run();

  // And make good on our promise
  return 0;
}
