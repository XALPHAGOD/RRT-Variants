import math 
import numpy as np #mathematical and logical operations on arrays
import matplotlib.pyplot as plt
import time
start_time = time.time()
import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtConnect:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.V1 = [self.s_start]
        self.V2 = [self.s_goal]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.genRandNode(self.s_goal, self.goal_sample_rate)
            node_near = self.closestNeighbour(self.V1, node_rand)
            node_new = self.genNewNode(node_near, node_rand)

            if node_new and not self.utils.isCollisionOccurs(node_near, node_new):
                self.V1.append(node_new)
                node_near_prim = self.closestNeighbour(self.V2, node_new)
                node_new_prim = self.genNewNode(node_near_prim, node_new)

                if node_new_prim and not self.utils.isCollisionOccurs(node_new_prim, node_near_prim):
                    self.V2.append(node_new_prim)

                    while True:
                        node_new_prim2 = self.genNewNode(node_new_prim, node_new)
                        if node_new_prim2 and not self.utils.isCollisionOccurs(node_new_prim2, node_new_prim):
                            self.V2.append(node_new_prim2)
                            node_new_prim = self.change_node(node_new_prim, node_new_prim2)
                        else:
                            break

                        if self.is_node_same(node_new_prim, node_new):
                            break

                if self.is_node_same(node_new_prim, node_new):
                    finalPath= self.extractPath(node_new, node_new_prim)
                    print("--- %s seconds ---" % (time.time() - start_time))
                    return finalPath

            if len(self.V2) < len(self.V1):
                list_mid = self.V2
                self.V2 = self.V1
                self.V1 = list_mid

        return None

    @staticmethod
    def change_node(node_new_prim, node_new_prim2):
        node_new = Node((node_new_prim2.x, node_new_prim2.y))
        node_new.parent = node_new_prim

        return node_new

    @staticmethod
    def is_node_same(node_new_prim, node_new):
        if node_new_prim.x == node_new.x and \
                node_new_prim.y == node_new.y:
            return True

        return False

    def genRandNode(self, sample_goal, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return sample_goal

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

    @staticmethod
    def extractPath(node_new, node_new_prim):
        path1 = [(node_new.x, node_new.y)]
        node_now = node_new

        while node_now.parent is not None:
            node_now = node_now.parent
            path1.append((node_now.x, node_now.y))

        path2 = [(node_new_prim.x, node_new_prim.y)]
        node_now = node_new_prim

        while node_now.parent is not None:
            node_now = node_now.parent
            path2.append((node_now.x, node_now.y))

        path=list(list(reversed(path1)) + path2)
        cost=0
        for i in range (len(path)-1):
            cost=cost+math.sqrt((path[i][0]-path[i+1][0])**2 +(path[i][1]-path[i+1][1])**2)

        print("cost= ",cost)
        print("no. of nodes ",len(path))


        return list(list(reversed(path1)) + path2)

    @staticmethod
    def getDistAngle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (46, 26)  # Goal node

    rrt_conn = RrtConnect(x_start, x_goal, 0.5, 0.1, 10000)
    path = rrt_conn.planning()
    rrt_conn.plotting.animationConnect(rrt_conn.V1, rrt_conn.V2, path, "RRT CONNECT, Iter= 10000")


if __name__ == '__main__':
    main()
