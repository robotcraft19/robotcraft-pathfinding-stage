#!/usr/bin/env python
import rospy
import time
from map_loader import MapLoader
from astar import PathFinder
from astar_v2 import AStar
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import atan2, pi
from goto_controller import Pose, GotoController
import itertools


class Cell:
    resolution = None
    start = None

    def __init__(self, row, column):
        self.row = row
        self.column = column
        self.rel_row = self.row - Cell.start.pos_x
        self.rel_column = self.column - Cell.start.pos_y

    def pose(self):
        # Check if cell has any walls as neighbours
        neighbors_matrix = GlobalPlanner.matrix[
            self.row - 1 : self.row + 2, self.column - 1 : self.column + 2
        ]
        diff_x = 0
        diff_y = 0

        if 1 in neighbors_matrix:  # check if any walls at all
            if 1 in neighbors_matrix[0, :]:
                diff_y -= Cell.resolution / 2  # move downwards if any wall in top row
            if 1 in neighbors_matrix[2, :]:
                diff_y += Cell.resolution / 2  # move upwards if any wall in bottom row
            if 1 in neighbors_matrix[:, 0]:
                diff_x += (
                    Cell.resolution / 2
                )  # move to the right if any wall in left column
            if 1 in neighbors_matrix[:, 2]:
                diff_x -= (
                    Cell.resolution / 2
                )  # move to the left if any wall in right column
            rospy.logwarn(
                "Path passing close to wall, using diff matrix: [%s, %s]",
                diff_x,
                diff_y,
            )

        # Set default Pose to center of cell instead of top-left corner and add calculated diffs
        return Pose(
            (self.rel_column * Cell.resolution) + Cell.resolution / 2 + diff_x,
            (-self.rel_row * Cell.resolution) - Cell.resolution / 2 + diff_y,
            0,
        )


class GlobalPlanner:
    matrix = None

    def __init__(self, precision):
        rospy.init_node("global_planner", anonymous=True)

        # Try to load parameters from launch file, otherwise set to None
        try:
            positions = rospy.get_param("~position")
            startX, startY = positions["startX"], positions["startY"]
            targetX, targetY = positions["targetX"], positions["targetY"]
            start = (startX, startY)
            target = (targetX, targetY)
        except:
            start, target = None, None

        # Load maze matrix
        # do not crop if target outside of maze
        self.map_loader = MapLoader(start, target)
        self.map_matrix = self.map_loader.loadMap()
        GlobalPlanner.matrix = self.map_matrix
        Cell.resolution = self.map_loader.occupancy_grid.info.resolution

        self.precision = precision

        # Calculate path
        t = time.time()
        self.path_finder = AStar(self.map_matrix)
        raw_path = self.path_finder.search()
        print(time.time() - t)
        Cell.start = self.path_finder.start
        self.path = [Cell(r, c) for r, c in raw_path]
        self.goal = self.path[0].pose()
        self.path_index = 0
        self.pose = Pose(0, 0, 0)

        # Setup publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.pid_pub = rospy.Publisher("/pid_err", Float64, queue_size=10)

        # Setup subscribers
        _ = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.pose.theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
        )

    def next_pose(self):
        try:
            self.path_index += 1
            self.goal = self.path[self.path_index].pose()
            rospy.loginfo("Moving to next pose: [%s, %s]", self.goal.x, self.goal.y)
        except IndexError:
            rospy.logwarn("REACHED END OF PATH!")

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        speed = Twist()
        goto = GotoController()
        goto.set_max_linear_acceleration(.05)
        goto.set_max_angular_acceleration(.2)
        goto.set_forward_movement_only(True)
        while not rospy.is_shutdown():

            speed_pose = goto.get_velocity(self.pose, self.goal, 0)
            # self.pid_pub.publish(goto.desiredAngVel)
            speed.linear.x = speed_pose.xVel
            speed.angular.z = speed_pose.thetaVel
            self.cmd_vel_pub.publish(speed)
            if goto.get_goal_distance(self.pose, self.goal) <= .05: # 5 cm precision
                self.next_pose()
            rate.sleep()


if __name__ == "__main__":
    try:
        controller = GlobalPlanner(0.02)
        time.sleep(5)
        controller.run()

    except rospy.ROSInterruptException:
        pass
