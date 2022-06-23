import math
import numpy as np
import env


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Utils:
    def __init__(self):
        self.env = env.Env()

        self.delta = 0.5
        self.obsCircle = self.env.obsCircle
        self.obsRectangle = self.env.obsRectangle
        self.obsBoundary = self.env.obsBoundary

    def updateObs(self, obs_cir, obs_bound, obs_rec):
        self.obsCircle = obs_cir
        self.obsBoundary = obs_bound
        self.obsRectangle = obs_rec

    def getObsVertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obsRectangle:
            vertex_list = [
                [ox - delta, oy - delta],
                [ox + w + delta, oy - delta],
                [ox + w + delta, oy + h + delta],
                [ox - delta, oy + h + delta],
            ]
            obs_list.append(vertex_list)

        return obs_list

    def isIntersectRec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.getDist(start, shot)
            dist_seg = self.getDist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def isIntersectCircle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.getDist(shot, Node(a)) <= r + delta:
                return True

        return False

    def isCollisionOccurs(self, start, end):
        if self.isInsideObs(start) or self.isInsideObs(end):
            return True

        o, d = self.getRay(start, end)
        obs_vertex = self.getObsVertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.isIntersectRec(start, end, o, d, v1, v2):
                return True
            if self.isIntersectRec(start, end, o, d, v2, v3):
                return True
            if self.isIntersectRec(start, end, o, d, v3, v4):
                return True
            if self.isIntersectRec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obsCircle:
            if self.isIntersectCircle(o, d, [x, y], r):
                return True

        return False

    def isInsideObs(self, node):
        delta = self.delta

        for (x, y, r) in self.obsCircle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obsRectangle:
            if (
                0 <= node.x - (x - delta) <= w + 2 * delta
                and 0 <= node.y - (y - delta) <= h + 2 * delta
            ):
                return True

        for (x, y, w, h) in self.obsBoundary:
            if (
                0 <= node.x - (x - delta) <= w + 2 * delta
                and 0 <= node.y - (y - delta) <= h + 2 * delta
            ):
                return True

        return False

    @staticmethod
    def getRay(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def getDist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
