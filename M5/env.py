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
        obsRectangle = []
        return obsRectangle

    @staticmethod
    def obsCircle():
        obs_cir = [[35, 7, 6], [15, 23, 7], [21, 9, 7], [29, 21, 5], [40, 19, 5], [7, 10, 5]]

        return obs_cir


# x_start = (2, 2)
# x_goal = (46, 26)