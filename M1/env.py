class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obsBoundary = self.obsBoundary()
        self.obsCircle = self.obsCircle()
        self.obsRectangle = self.obsRectangle()

    @staticmethod
    def obsBoundary():
        obsBoundary = [[0, 0, 1, 30], [0, 30, 50, 1], [1, 0, 50, 1], [50, 1, 1, 30]]
        return obsBoundary

    @staticmethod
    def obsRectangle():
        obsRectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obsRectangle

    @staticmethod
    def obsCircle():
        obs_cir = [[7, 12, 3], [46, 20, 2], [15, 5, 2], [37, 7, 3], [37, 23, 3]]

        return obs_cir


# x_start = (2, 2)
# x_goal = (49, 24)