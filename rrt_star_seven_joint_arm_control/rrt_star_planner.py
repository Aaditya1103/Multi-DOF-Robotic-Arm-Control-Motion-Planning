"""
RRT* planner for a 7-DOF robotic manipulator
Author: Aaditya Sakhardande
"""

import math
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from n_joint_arm_3d.NLinkArm3d import NLinkArm

ENABLE_ANIMATION = True
DEBUG = False


class RobotArm(NLinkArm):
    def forward_kinematics_points(self, joint_angles):
        self.set_joint_angles(joint_angles)

        xs, ys, zs = [], [], []
        T = np.eye(4)

        xs.append(T[0, 3])
        ys.append(T[1, 3])
        zs.append(T[2, 3])

        for link in self.link_list:
            T = T @ link.transformation_matrix()
            xs.append(T[0, 3])
            ys.append(T[1, 3])
            zs.append(T[2, 3])

        return xs, ys, zs


class RRTStar:
    """RRT* implementation for joint-space planning"""

    class TreeNode:
        def __init__(self, config):
            self.x = config
            self.parent = None
            self.cost = 0.0

    def __init__(self, start, goal, robot, obstacle_list, rand_area,
                 expand_dis=0.30,
                 path_resolution=0.1,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0):

        self.start = self.TreeNode(start)
        self.goal = self.TreeNode(goal)

        self.dimension = len(start)
        self.q_min, self.q_max = rand_area
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.connect_circle_dist = connect_circle_dist

        self.robot = robot
        self.obstacles = obstacle_list
        self.nodes = []

        if ENABLE_ANIMATION:
            self.ax = plt.axes(projection="3d")

    def plan(self, animation=False, search_full=False):
        self.nodes = [self.start]

        for i in range(self.max_iter):
            if DEBUG:
                print(f"Iteration {i}, Nodes: {len(self.nodes)}")

            sample = self.sample_configuration()
            nearest_idx = self.closest_node_index(self.nodes, sample)
            new_node = self.extend(self.nodes[nearest_idx], sample, self.expand_dis)

            if self.is_collision_free(new_node):
                nearby = self.get_nearby_nodes(new_node)
                new_node = self.select_parent(new_node, nearby)

                if new_node:
                    self.nodes.append(new_node)
                    self.rewire_tree(new_node, nearby)

            if animation and i % 5 == 0 and self.dimension <= 3:
                self.render(sample)

            if not search_full and new_node:
                best = self.find_goal_node()
                if best is not None:
                    return self.extract_path(best)

        best = self.find_goal_node()
        if best is not None:
            return self.extract_path(best)

        return None

    def sample_configuration(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            q = np.random.uniform(self.q_min, self.q_max, self.dimension)
            return self.TreeNode(q)
        return self.TreeNode(self.goal.x)

    def extend(self, from_node, to_node, max_length=float("inf")):
        new = self.TreeNode(list(from_node.x))
        dist, _, _ = self.distance_and_angles(new, to_node)

        new.path_x = [list(new.x)]

        max_length = min(max_length, dist)
        steps = math.floor(max_length / self.path_resolution)

        start = np.array(from_node.x)
        target = np.array(to_node.x)
        direction = (target - start) / np.linalg.norm(target - start)

        for _ in range(steps):
            new.x += direction * self.path_resolution
            new.path_x.append(list(new.x))

        if np.linalg.norm(np.array(new.x) - target) <= self.path_resolution:
            new.path_x.append(list(to_node.x))

        new.parent = from_node
        return new

    def select_parent(self, node, near_indices):
        if not near_indices:
            return None

        costs = []
        for idx in near_indices:
            candidate = self.extend(self.nodes[idx], node)
            if candidate and self.is_collision_free(candidate):
                costs.append(self.compute_cost(self.nodes[idx], node))
            else:
                costs.append(float("inf"))

        best_cost = min(costs)
        if best_cost == float("inf"):
            return None

        best_idx = near_indices[costs.index(best_cost)]
        node = self.extend(self.nodes[best_idx], node)
        node.parent = self.nodes[best_idx]
        node.cost = best_cost

        return node

    def rewire_tree(self, node, near_indices):
        for idx in near_indices:
            neighbor = self.nodes[idx]
            candidate = self.extend(node, neighbor)

            if not candidate:
                continue

            candidate.cost = self.compute_cost(node, neighbor)

            if self.is_collision_free(candidate) and neighbor.cost > candidate.cost:
                self.nodes[idx] = candidate
                self.update_descendants(node)

    def update_descendants(self, parent):
        for n in self.nodes:
            if n.parent == parent:
                n.cost = self.compute_cost(parent, n)
                self.update_descendants(n)

    def find_goal_node(self):
        dists = [np.linalg.norm(np.array(n.x) - np.array(self.goal.x)) for n in self.nodes]
        close_nodes = [i for i, d in enumerate(dists) if d <= self.expand_dis]

        valid = []
        for idx in close_nodes:
            test = self.extend(self.nodes[idx], self.goal)
            if self.is_collision_free(test):
                valid.append(idx)

        if not valid:
            return None

        return min(valid, key=lambda i: self.nodes[i].cost)

    def extract_path(self, idx):
        path = [self.goal.x]
        node = self.nodes[idx]
        while node.parent:
            path.append(node.x)
            node = node.parent
        path.append(node.x)
        return path

    def compute_cost(self, a, b):
        d, _, _ = self.distance_and_angles(a, b)
        return a.cost + d

    def get_nearby_nodes(self, node):
        n = len(self.nodes) + 1
        radius = self.connect_circle_dist * math.sqrt(math.log(n) / n)
        radius = min(radius, self.expand_dis)

        dists = [(np.linalg.norm(np.array(n_.x) - np.array(node.x)))**2 for n_ in self.nodes]
        return [i for i, d in enumerate(dists) if d <= radius**2]

    def is_collision_free(self, node):
        if node is None:
            return False

        for ox, oy, oz, r in self.obstacles:
            for q in node.path_x:
                xs, ys, zs = self.robot.forward_kinematics_points(q)
                for x, y, z in zip(xs, ys, zs):
                    if (ox - x)**2 + (oy - y)**2 + (oz - z)**2 <= r**2:
                        return False
        return True

    def render(self, sample=None):
        plt.cla()
        self.ax.axis([-1, 1, -1, 1, -1, 1])
        self.ax.set_zlim(0, 1)
        self.ax.grid(True)

        for ox, oy, oz, r in self.obstacles:
            self.draw_sphere(ox, oy, oz, r)

        if sample:
            self.ax.plot([sample.x[0]], [sample.x[1]], [sample.x[2]], "^k")

        for node in self.nodes:
            if node.parent:
                p = np.array(node.path_x)
                self.ax.plot(p[:, 0], p[:, 1], p[:, 2], "-g")

        self.ax.plot([self.start.x[0]], [self.start.x[1]], [self.start.x[2]], "xr")
        self.ax.plot([self.goal.x[0]], [self.goal.x[1]], [self.goal.x[2]], "xr")
        plt.pause(0.01)

    def draw_sphere(self, x, y, z, r):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xs = x + r * np.cos(u) * np.sin(v)
        ys = y + r * np.sin(u) * np.sin(v)
        zs = z + r * np.cos(v)
        self.ax.plot_wireframe(xs, ys, zs, color="k")

    @staticmethod
    def distance_and_angles(a, b):
        delta = np.array(b.x) - np.array(a.x)
        d = np.linalg.norm(delta)
        phi = math.atan2(delta[1], delta[0])
        theta = math.atan2(math.hypot(delta[0], delta[1]), delta[2])
        return d, phi, theta

    @staticmethod
    def closest_node_index(nodes, target):
        dists = [np.linalg.norm(np.array(n.x) - np.array(target.x))**2 for n in nodes]
        return dists.index(min(dists))


def main():
    print("Running:", __file__)

    panda = RobotArm([
        [0.,  math.pi/2., 0.,      0.333],
        [0., -math.pi/2., 0.,      0.],
        [0.,  math.pi/2., 0.0825,  0.316],
        [0., -math.pi/2., -0.0825, 0.],
        [0.,  math.pi/2., 0.,      0.384],
        [0.,  math.pi/2., 0.088,   0.],
        [0.,  0.,         0.,      0.107],
    ])

    obstacles = [
        (-0.3, -0.3, 0.7, 0.1),
        (0.0,  -0.3, 0.7, 0.1),
        (0.2,  -0.1, 0.3, 0.15),
    ]

    q_start = [0.0] * len(panda.link_list)
    q_goal = [1.5] * len(panda.link_list)

    planner = RRTStar(
        start=q_start,
        goal=q_goal,
        rand_area=[0, 2],
        max_iter=200,
        robot=panda,
        obstacle_list=obstacles
    )

    path = planner.plan(animation=ENABLE_ANIMATION)

    if path is None:
        print("Path not found")
        return

    print("Path found")

    if ENABLE_ANIMATION:
        planner.render()
        for q in path:
            xs, ys, zs = panda.forward_kinematics_points(q)
            planner.ax.plot(xs, ys, zs, "o-", color="gray", ms=4)
            plt.pause(0.1)
        plt.show()


if __name__ == "__main__":
    main()
