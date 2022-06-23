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
            [10, 7, 15, 1],
            [25, 12, 15, 1],
            [10, 17, 15, 1],
            [25, 22, 15, 1],
        ]
        return obsRectangle

    @staticmethod
    def obsCircle():
        obs_cir = []

        return obs_cir


# x_start = (25, 4)
# x_goal = (25, 26)