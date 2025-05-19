import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
import numpy as np

from utils.grid import Grid
from utils.environment import Environment
from utils.cases import TestCase
from utils.utils import distance

from time import time


class Node:
    """ Standard A* node. """

    def __init__(self, cell_id):

        self.cell_id = cell_id
        self.g = None
        self.f = None
        self.parent = None
    
    def __eq__(self, other):

        return self.cell_id == other.cell_id
    
    def __hash__(self):

        return hash((self.cell_id))


class Params:
    """ Store the computed costs. """

    def __init__(self, cell_id, g):

        self.cell_id = cell_id
        self.g = g
    
    def __eq__(self, cell_id):

        return self.cell_id == cell_id
    
    def __hash__(self):

        return hash((self.cell_id))


class Astar:
    """ Standard A*. """

    def __init__(self, grid, start):

        self.grid = grid
        self.start = self.grid.to_cell_id(start)
        self.table = []
    
    def heuristic(self, p1, p2):
        """ Simple Manhattan distance  heuristic. """

        return abs(p2[0]-p1[0]) + abs(p2[1]-p1[1])
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.cell_id))
            node = node.parent
        
        route.append((self.start))
        
        return list(reversed(route))
    
    def search_path(self, goal):
        """ Search the path by astar. """

        goal = self.grid.to_cell_id(goal)

        if goal in self.table:
            return self.table[self.table.index(goal)].g

        root = Node(self.start)
        root.g = 0
        root.f = root.g + self.heuristic(self.start, goal)

        closed_ = []
        open_ = [root]

        while open_:

            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            if best.cell_id == goal:
                # route = self.backtracking(best)
                self.table.append(Params(goal, best.g))
                return best.g

            nbs = self.grid.get_neighbors(best.cell_id)

            for nb in nbs:
                child = Node(nb)
                child.g = best.g + 1
                child.f = child.g + self.heuristic(nb, goal)
                child.parent = best

                if child in closed_:
                    continue

                if child not in open_:
                    open_.append(child)
                
                elif child.g < open_[open_.index(child)].g:
                    open_.remove(child)
                    open_.append(child)
        
        return None