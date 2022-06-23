import math
import numpy as np
import time
import env, plotting, utils, queue


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStar:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

    def planning(self):
        start_time = time.time()
        for k in range(1,self.iter_max+1):
            node_rand = self.genRandNode(self.goal_sample_rate)
            node_near = self.closestNeighbour(self.vertex, node_rand)
            node_new = self.genNewNode(node_near, node_rand)

            if k % 1000 == 0:
                print(k," iters")

            if node_new and not self.utils.isCollisionOccurs(node_near, node_new):
                neighbor_index = self.findNearestNeighbours(node_new)
                self.vertex.append(node_new)

                if neighbor_index:
                    self.chooseParent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

        index = self.chooseGoalParent()
        if index != -1:
            self.path = self.extractPath(self.vertex[index])
            if len(self.path) == 1:
                print("No path found")
            else:
                print("cost= ", self.cost(self.vertex[index]))
                print("no. of nodes ",len(self.path))
                print("--- %s seconds ---" % (time.time() - start_time))

        else:
            print("No path found")

        self.plotting.animation(self.vertex, self.path, "RRT*, Iter = " + str(self.iter_max))

    def genNewNode(self, node_start, node_goal):
        dist, theta = self.getDistAngle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def chooseParent(self, node_new, neighbor_index):
        cost = [self.getNewCost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.getNewCost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def chooseGoalParent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.isCollisionOccurs(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]

        return  -1

    def getNewCost(self, node_start, node_end):
        dist, _ = self.getDistAngle(node_start, node_end)

        return self.cost(node_start) + dist

    def genRandNode(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def findNearestNeighbours(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.isCollisionOccurs(node_new, self.vertex[ind])]

        return dist_table_index

    @staticmethod
    def closestNeighbour(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def updateCost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.getNewCost(node, node_c)
                OPEN.put(node_c)

    def extractPath(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    @staticmethod
    def getDistAngle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (46, 26)  # Goal node

    rrt_star = RrtStar(x_start, x_goal, 0.5, 0.10, 10, 10000)
    rrt_star.planning()


if __name__ == '__main__':
    main()
