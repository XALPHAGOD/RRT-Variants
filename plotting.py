import matplotlib.pyplot as plt
import matplotlib.patches as patches
import env


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = env.Env()
        self.obs_bound = self.env.obsBoundary
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle

    def animation(self, nodelist, path, name, animation=False):
        self.plotGrid(name)
        self.plotVisited(nodelist, animation)
        self.plotPath(path)

    def animationConnect(self, V1, V2, path, name):
        self.plotGrid(name)
        self.plotVisitedConnect(V1, V2)
        self.plotPath(path)

    def plotGrid(self, name):
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="black", fill=True
                )
            )

        for (ox, oy, w, h) in self.obsRectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="gray", fill=True
                )
            )

        for (ox, oy, r) in self.obsCircle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r, edgecolor="black", facecolor="gray", fill=True
                )
            )

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plotVisited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect(
                        "key_release_event",
                        lambda event: [exit(0) if event.key == "escape" else None],
                    )
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plotVisitedConnect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )

            if k % 10 == 0:
                plt.pause(0.001)

    @staticmethod
    def plotPath(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], "-r", linewidth=2)
            plt.pause(0.01)
        plt.show()
