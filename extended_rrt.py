import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class ExtendedRrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.waypoint_sample_rate = waypoint_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()
        self.fig, self.ax = plt.subplots()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

        self.path = []
        self.waypoint = []

    def planning(self):
        start_time=time.time()

        for i in range(self.iter_max):
            node_rand = self.genRandNode(self.goal_sample_rate)
            node_near = self.closestNeighbour(self.vertex, node_rand)
            node_new = self.genNewNode(node_near, node_rand)

            if node_new and not self.utils.isCollisionOccurs(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.getDistAngle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.genNewNode(node_new, self.s_goal)
                    print("time: %s seconds" % (time.time()-start_time))
                    
                    path = self.extractPath(node_new)

                    cost=0
                    for i in range(len(path)-1):
                        cost=cost+math.sqrt((path[i][0]-path[i+1][0])**2+ (path[i][1]-path[i+1][1])**2)
                    print(f"nodes: {len(path)} ,cost: {cost}")

                    self.plotGrid("Extended RRT, Iter= 10000")
                    self.plotVisited()
                    self.plotPath(path)
                    self.path = path
                    self.waypoint = self.extractWaypoint(node_new)
                    self.fig.canvas.mpl_connect('button_press_event', self.on_press)
                    plt.show()

                    return

        return None

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > 50 or y < 0 or y > 30:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Add circle obstacle at: s =", x, ",", "y =", y)
            self.obsCircle.append([x, y, 2])
            self.utils.updateObs(self.obsCircle, self.obsBoundary, self.obsRectangle)
            path, waypoint = self.replanning()
            
            if len(path) > 0 :
                self.path = path
                self.waypoint = waypoint
            else:
                print("No path found")
            
            plt.cla()
            self.plotGrid("Extended_RRT")
            self.plotPath(self.path, color='blue')
            self.plotVisited()
            self.plotPath(path)
            self.fig.canvas.draw_idle()

    def replanning(self):
        start_time=time.time()
        self.vertex = [self.s_start]

        for i in range(self.iter_max):
            node_rand = self.genRandNodeReplanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.closestNeighbour(self.vertex, node_rand)
            node_new = self.genNewNode(node_near, node_rand)

            if node_new and not self.utils.isCollisionOccurs(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.getDistAngle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.genNewNode(node_new, self.s_goal)
                    print("time: %s seconds" % (time.time()-start_time))
                    path = self.extractPath(node_new)

                    cost=0
                    for i in range(len(path)-1):
                        cost=cost+math.sqrt((path[i][0]-path[i+1][0])**2+ (path[i][1]-path[i+1][1])**2)
                    print(f"nodes: {len(path)} ,cost: {cost}")

                    waypoint = self.extractWaypoint(node_new)

                    return (path, waypoint)

        return [], []

    def genRandNode(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def genRandNodeReplanning(self, goal_sample_rate, waypoint_sample_rate):
        delta = self.utils.delta
        p = np.random.random()

        if p < goal_sample_rate:
            return self.s_goal
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:
            return self.waypoint[np.random.randint(0, len(self.path) - 1)]
        else:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))


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

    def extractWaypoint(self, node_end):
        waypoint = [self.s_goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            waypoint.append(node_now)

        return waypoint

    @staticmethod
    def getDistAngle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def plotGrid(self, name):

        for (ox, oy, w, h) in self.obsBoundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obsRectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obsCircle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.s_start.x, self.s_start.y, "bs", linewidth=3)
        plt.plot(self.s_goal.x, self.s_goal.y, "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    def plotVisited(self):
        animation = True
        if animation:
            count = 0
            for node in self.vertex:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    # if count % 10 == 0:
                    #     plt.pause(0.001)
        else:
            for node in self.vertex:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plotPath(path, color='red'):
        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color=color)
        # plt.pause(0.001)


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (46, 26)  # Goal node

    errt = ExtendedRrt(x_start, x_goal, 0.5, 0.1, 0.6, 10000)
    errt.planning()


if __name__ == '__main__':
    main()