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
            [28, 11, 1, 4],
            [22, 11, 6, 1],
            [21, 11, 1, 9],
            [21, 20, 13, 1],
            [34, 6, 1, 15],
            [16, 5, 19, 1],
            [15, 5, 1, 21],
            [15, 26, 20, 1]
        ]
        return obsRectangle

    @staticmethod
    def obsCircle():
        obs_cir = []

        return obs_cir


# x_start = (25, 16)
# x_goal = (40, 24)