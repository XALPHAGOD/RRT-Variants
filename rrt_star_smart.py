import math
import random
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot
import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStarSmart:
    def __init__(
        self, x_start, x_goal, step_len, goal_sample_rate, biasing_radius, iter_max
    ):
        self.start_node = Node(x_start)
        self.goal_node = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.biasing_radius = biasing_radius
        self.iter_max = iter_max

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.fig, self.ax = plt.subplots()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

        self.vertices = [self.start_node]
        self.beacons = []
        self.beacons_radius = 2
        self.direct_cost_old = np.inf
        self.obs_vertex = self.utils.getObsVertex()
        self.path = None
        self.biasingRatio = 5
        self.InitPathFlag = False

    def planning(self):
        start_time = time.time()
        n = 0
        self.reformObsVertex()

        for k in range(1,self.iter_max+1):
            if k % 1000 == 0:
                print(k," iters")

            if (k - n) % self.biasingRatio == 0 and len(self.beacons) > 0:
                rand_node = self.genRandNode(self.beacons)
            else:
                rand_node = self.genRandNode()

            nearest_node = self.closestNeighbour(self.vertices, rand_node)
            new_node = self.genNewNode(nearest_node, rand_node)
            if (
                new_node
                and new_node.x != self.goal_node.x
                and new_node.y != self.goal_node.y
                and not self.utils.isCollisionOccurs(nearest_node, new_node)
            ):
                X_near = self.findNearestNeighbours(self.vertices, new_node)
                self.vertices.append(new_node)

                if X_near:
                    # choose parent
                    cost_list = [
                        self.cost(x_near) + self.lineCost(x_near, new_node)
                        for x_near in X_near
                    ]
                    new_node.parent = X_near[int(np.argmin(cost_list))]

                    # rewire
                    c_min = self.cost(new_node)
                    for x_near in X_near:
                        c_near = self.cost(x_near)
                        c_new = c_min + self.lineCost(new_node, x_near)
                        if c_new < c_near:
                            x_near.parent = new_node

                if self.pathFound(new_node):
                    if not self.InitPathFlag:
                        self.InitPathFlag = True
                        n = k

                    self.pathOptimization(new_node)

                # if k % 5 == 0:
                #     self.animation()

        self.path = self.extractPath()
        if len(self.path) == 1:
            print("No path found")
        else:
            print("cost= ", self.cost(self.goal_node))
            print("no. of nodes ",len(self.path))
            print("--- %s seconds ---" % (time.time() - start_time))
        self.animation()
        plt.plot([x for x, _ in self.path], [y for _, y in self.path], "-r")
        plt.pause(0.01)
        plt.show()

    def pathOptimization(self, node):
        direct_cost_new = 0.0
        node_end = self.goal_node

        while node.parent:
            node_parent = node.parent
            if not self.utils.isCollisionOccurs(node_parent, node_end):
                node_end.parent = node_parent
            else:
                direct_cost_new += self.lineCost(node, node_end)
                node_end = node

            node = node_parent

        direct_cost_new += self.lineCost(node, node_end)
        if direct_cost_new < self.direct_cost_old:
            self.direct_cost_old = direct_cost_new
            self.updateBeacons()

    def updateBeacons(self):
        node = self.goal_node.parent
        beacons = []

        while node.parent:
            near_vertex = [
                v
                for v in self.obs_vertex
                if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9
            ]
            if len(near_vertex) > 0:
                for v in near_vertex:
                    beacons.append(v)

            node = node.parent

        self.beacons = beacons

    def reformObsVertex(self):
        obs_vertex = []

        for obs in self.obs_vertex:
            for vertex in obs:
                obs_vertex.append(vertex)

        self.obs_vertex = obs_vertex

    def genNewNode(self, x_start, x_goal):
        dist, theta = self.getDistAngle(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node(
            (x_start.x + dist * math.cos(theta), x_start.y + dist * math.sin(theta))
        )
        node_new.parent = x_start
        return node_new

    def findNearestNeighbours(self, nodelist, node):
        n = len(self.vertices) + 1
        r = self.biasing_radius * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        X_near = [
            nodelist[ind]
            for ind in range(len(dist_table))
            if dist_table[ind] <= r ** 2
            and not self.utils.isCollisionOccurs(node, nodelist[ind])
        ]
        return X_near

    def genRandNode(self, goal=None):
        if goal is None:
            delta = self.utils.delta
            goal_sample_rate = self.goal_sample_rate

            if np.random.random() > goal_sample_rate:
                return Node(
                    (
                        np.random.uniform(
                            self.x_range[0] + delta, self.x_range[1] - delta
                        ),
                        np.random.uniform(
                            self.y_range[0] + delta, self.y_range[1] - delta
                        ),
                    )
                )

            return self.goal_node
        else:
            R = self.beacons_radius
            r = random.uniform(0, R)
            theta = random.uniform(0, 2 * math.pi)
            ind = random.randint(0, len(goal) - 1)

            return Node(
                (goal[ind][0] + r * math.cos(theta), goal[ind][1] + r * math.sin(theta))
            )

    def extractPath(self):
        path = []
        node = self.goal_node

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.start_node.x, self.start_node.y])

        return path

    def pathFound(self, node):
        if (
            node.x != self.goal_node.x
            and node.y != self.goal_node.y
            and self.lineCost(node, self.goal_node) <= self.step_len
            and not self.utils.isCollisionOccurs(node, self.goal_node)
        ):
            return True

        return False

    @staticmethod
    def closestNeighbour(nodelist, n):
        return nodelist[
            int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2 for nd in nodelist]))
        ]

    @staticmethod
    def lineCost(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    @staticmethod
    def cost(node):
        cost = 0.0
        if node.parent is None:
            return cost

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    @staticmethod
    def getDistAngle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def animation(self):
        plt.cla()
        self.plotGrid("RRT* Smart, Iter = " + str(self.iter_max))
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )

        for node in self.vertices:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        if self.beacons:
            theta = np.arange(0, 2 * math.pi, 0.1)
            r = self.beacons_radius

            for v in self.beacons:
                x = v[0] + r * np.cos(theta)
                y = v[1] + r * np.sin(theta)
                plt.plot(x, y, linestyle="--", linewidth=1, color="darkorange")

        plt.pause(0.01)

    def plotGrid(self, name):

        for (ox, oy, w, h) in self.obsBoundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="black", fill=True
                )
            )

        for (ox, oy, w, h) in self.obsRectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="gray", fill=True
                )
            )

        for (ox, oy, r) in self.obsCircle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r, edgecolor="black", facecolor="gray", fill=True
                )
            )

        plt.plot(self.start_node.x, self.start_node.y, "bs", linewidth=1)
        plt.plot(self.goal_node.x, self.goal_node.y, "rs", linewidth=1)

        plt.title(name)
        plt.axis("equal")


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (46, 26)  # Goal node

    rrt = RrtStarSmart(x_start, x_goal, 0.5, 0.1, 10, 10000)
    rrt.planning()


if __name__ == "__main__":
    main()
