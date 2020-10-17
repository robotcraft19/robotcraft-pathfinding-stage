import time
from math import sqrt
from sys import stdout
import numpy as np
import random
from termcolor import colored
import os

# 1 for manhattan, 0 for euclidean
HEURISTIC = 0
DIRECTIONS = [
                [1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [1, 1, sqrt(2)],
                [-1, 1, sqrt(2)],
                [1, -1, sqrt(2)],
                [-1, -1, sqrt(2)]
        ]

class Node:
    """
    >>> k = Node(0, 0, 4, 3, 0, None)
    >>> k.calculate_heuristic()
    5.0
    >>> n = Node(1, 4, 3, 4, 2, None)
    >>> n.calculate_heuristic()
    2.0
    >>> l = [k, n]
    >>> n == l[0]
    False
    >>> l.sort()
    >>> n == l[0]
    True
    """

    def __init__(self, pos_x, pos_y, goal_x, goal_y, g_cost, is_closed, is_open, parent):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos = (pos_y, pos_x)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.is_closed = is_closed
        self.is_open = is_open
        self.g_cost = g_cost
        self.parent = parent
        self.h_cost = self.calculate_heuristic()
        self.f_cost = self.g_cost + self.h_cost + self.calculate_wall_bias()

    def calculate_wall_bias(self):
        wall_penalty = 0
        for dir in DIRECTIONS:
            try:
                nb = Node.matrix[self.pos_x + dir[0]][self.pos_y + dir[1]]
                if nb == 1:
                    wall_penalty += 10.0
                for dir_dir in DIRECTIONS:
                    try:
                        nb_nb = Node.matrix[self.pos_x + dir[0] + dir_dir[0]][
                            self.pos_y + dir[1] + dir_dir[1]

                        ]
                        if nb_nb == 1:
                            wall_penalty += 5.0
                    except:
                        # out of bounds
                        pass
            except:
                # out of bounds
                pass

        return wall_penalty
      
    def calculate_heuristic(self):
        """
        Heuristic for the A*
        """
        dy = self.pos_x - self.goal_x
        dx = self.pos_y - self.goal_y
        if HEURISTIC == 1:
            return abs(dx) + abs(dy)
        else:
            return sqrt(dy ** 2 + dx ** 2)

    def __str__(self):
        print("Node : <Pos : ", self.pos, " >")

    def __lt__(self, other):
        return self.f_cost <= other.f_cost


class AStar:
    """
    >>> wd = World(10, 10, 0.2)
    >>> astar = AStar((0, 0), (len(wd.grid) - 1, len(wd.grid[0]) - 1), wd)
    >>> (astar.start.pos_y + wd.delta[3][0], astar.start.pos_x + wd.delta[3][1])
    (0, 1)
    >>> [x.pos for x in astar.get_successors(astar.start)]
    [(1, 0), (0, 1)]
    >>> (astar.start.pos_y + wd.delta[2][0], astar.start.pos_x + wd.delta[2][1])
    (1, 0)
    >>> astar.retrace_path(astar.start)
    [(0, 0)]
    >>> astar.search()  # doctest: +NORMALIZE_WHITESPACE
    [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (3, 3),
     (4, 3), (4, 4), (5, 4), (5, 5), (6, 5), (6, 6)]
    """

    def __init__(self, matrix):
        self.matrix = np.array(matrix)
        self.directions = DIRECTIONS
        goal = np.where(self.matrix == -2)
        self.target = Node(goal[0][0], goal[1][0], goal[0][0], goal[1][0], 99999, False, False, None)  # extract indices

        # Get matrix coordinates of initial robot position and create starting node
        start = np.where(self.matrix == -1)
        self.start = Node(start[0][0], start[1][0], goal[0][0], goal[1][0], 0, False, True, None)  # extract indices

        self.nodes = dict()
        self.nodes[self.start.pos] = self.start

        self.reached = False

    def search(self):
        while self.count_open_nodes() > 0:
            # Open Nodes are sorted using __lt__
            current_key = min([n for n in self.nodes if self.nodes[n].is_open], key=(lambda k: self.nodes[k].f_cost))
            current_node = self.nodes[current_key]

            if current_node.pos == self.target.pos:
                print(colored("Found path.", "green"))
                return self.retrace_path(current_node)

            current_node.is_closed = True
            current_node.is_open = False

            successors = self.get_successors(current_node)

            for child_node in successors:
                if child_node.pos in self.nodes:
                    if self.nodes[child_node.pos].is_closed:
                        continue
                    if not self.nodes[child_node.pos].is_open:
                        self.nodes[child_node.pos] = child_node
                    else:
                        if child_node.g_cost < self.nodes[child_node.pos].g_cost:
                            self.nodes[child_node.pos] = child_node
                else:
                    self.nodes[child_node.pos] = child_node
        print(colored("Path not found", "red"))
        return [self.start.pos]

    def count_open_nodes(self):
        i = 0
        for _, val in self.nodes.items():
            if val.is_open:
                i += 1
                break
        return i

    def get_successors(self, parent):
        """
        Returns a list of successors (both in the world and free spaces)
        """
        successors = []
        for action in self.directions:
            pos_x = parent.pos_x + action[1]
            pos_y = parent.pos_y + action[0]
            if not (0 <= pos_x < self.matrix.shape[1] and 0 <= pos_y < self.matrix.shape[0]):
                continue

            if self.matrix[pos_x][pos_y] == 1:
                continue

            successors.append(
                Node(
                    pos_x,
                    pos_y,
                    self.target.pos_x,
                    self.target.pos_y,
                    parent.g_cost + action[2],
                    False,
                    True,
                    parent,
                )
            )
        return successors

    def retrace_path(self, node):
        """
        Retrace the path from parents to parents until start node
        """
        current_node = node
        path = []
        while current_node is not None:
            path.append((current_node.pos_x, current_node.pos_y))
            current_node = current_node.parent
        path.reverse()
        self.write_map(path)
        return path

    def write_map(self, path, print_output=False):
        # Writes map to file and optionally prints it
        map = self.matrix.copy()
        with open(
            os.path.join(
                os.path.expanduser("~"),
                "catkin_ws/src/robotcraft-pathfinding-stage/scans/path_route.txt",
            ),
            "w",
        ) as f:
            for point in path:
                map[point[0], point[1]] = 7
            for row in map:
                for col in row:
                    f.write(str(int(col)))
                f.write("\n")
                