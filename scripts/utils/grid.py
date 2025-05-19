from math import sqrt

class Grid:
    """ Grid configuration. """

    def __init__(self, env, cell_size=0.25):

        self.env = env
        self.cell_size = cell_size

        self.n = int(self.env.lx / self.cell_size)
        self.m = int(self.env.ly / self.cell_size)

        self.cell_dia = sqrt(2*self.cell_size**2)

        self.get_obstacle_occupancy()
    
    def get_obstacle_occupancy(self):
        """ Fill grid with obstacles. """

        self.grid = [[0] * self.m for _ in range(self.n)]

        for ob in self.env.obs:
            x1, y1 = self.to_cell_id([ob.x, ob.y])
            x2, y2 = self.to_cell_id([ob.x+ob.w, ob.y+ob.h])

            if (ob.x+ob.w) % self.cell_size == 0:
                x2 -= 1
            
            if (ob.y+ob.h) % self.cell_size == 0:
                y2 -= 1

            for i in range(x1, x2+1):
                for j in range(y1, y2+1):
                    self.grid[i][j] = 1
    
    def to_cell_id(self, pt):
        """" Convert point into grid index. """

        x = min(int(pt[0] / self.cell_size), self.n-1)
        y = min(int(pt[1] / self.cell_size), self.m-1)

        return [x, y]
    
    def get_neighbors(self, cell_id):
        """ Get all the 4 adjacent cells. """

        x, y = cell_id
        nbs = []

        for p in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            if 0 <= x + p[0] < self.n and 0 <= y + p[1] < self.m:
                if self.grid[x + p[0]][y + p[1]] == 0:
                    nbs.append([x + p[0], y + p[1]])

        return nbs


