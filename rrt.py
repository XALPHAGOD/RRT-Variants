import math
import numpy as np
import time
import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

    def planning(self):
        start_time = time.time()
        for i in range(self.iter_max):
            node_rand = self.genRandNode(self.goal_sample_rate)
            node_near = self.closestNeighbour(self.vertex, node_rand)
            node_new = self.genNewNode(node_near, node_rand)

            if node_new and not self.utils.isCollisionOccurs(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.getDistAngle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.isCollisionOccurs(node_new, self.s_goal):
                    # self.genNewNode(node_new, self.s_goal)
                    self.s_goal.parent=node_new
                    finalPath= self.extractPath(node_new)
                    print("cost= ", self.Cost(self.s_goal))
                    print("no. of nodes ",len(finalPath))
                    print("--- %s seconds ---" % (time.time() - start_time))
                    return finalPath

        return None

    def genRandNode(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def closestNeighbour(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def genNewNode(self, node_start, node_end):
        dist, theta = self.getDistAngle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extractPath(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path
    
    @staticmethod
    def Cost(node):
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


def main():
    x_start = (25, 16)  # Starting node
    x_goal = (40, 24)  # Goal node

    rrt = Rrt(x_start, x_goal, 0.5, 0.1, 20000)
    path = rrt.planning()

    if path:
        rrt.plotting.animation(rrt.vertex, path, "RRT, Iter= 20000")
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
