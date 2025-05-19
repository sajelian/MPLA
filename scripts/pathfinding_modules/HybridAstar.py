#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from math import pi, sqrt, tan
import time
import argparse
import matplotlib.animation as animation
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point

from utils.environment import Environment
from utils.grid import Grid
from utils.car import SimpleCar
from utils.dubins_path import DubinsPath, ReedsSheppPath
from utils.obstacle import Obstacle

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 

OBSTACLES = [
            (0.0, 1.0, 0.5, 0.2),
            (3.31, 0, 1.9, 0.2),
            (1.2, 1.65, 0.2, 0.4),
            (2.2, 1.65, 0.4, 0.4),
            (1.35, 0.6, 0.5, 0.2),
            (3.06, 0.7, 0.5, 0.2),
            (3.71, 1.75, 0.4, 0.2)
        ]

ENV_LENGTH = 5.21
ENV_WIDTH = 2.75
GRID_SIZE = 0.05

CAR_L = 0.3
CAR_BACK_OFFSET = SimpleCar.CAR_LENGTH/2 + Obstacle.SAFE_DIS + 0.01
CAR_FRONT_OFFSET = SimpleCar.CAR_LENGTH/2 + Obstacle.SAFE_DIS + 0.01
CAR_SIDE_OFFSET = SimpleCar.CAR_WIDTH/2 + Obstacle.SAFE_DIS + 0.01

PLANNING_WAYPOINTS = [
    (0.2 + CAR_SIDE_OFFSET, 0.1 + CAR_BACK_OFFSET, pi/2), 
    (1.6, 0.6 - CAR_SIDE_OFFSET, 0), 
    (3.31, 0.9 + CAR_FRONT_OFFSET, -pi/2), 
    (3.21, 2.75 - CAR_FRONT_OFFSET, pi/2), 
    (5.21 - CAR_FRONT_OFFSET, 0.3 + CAR_SIDE_OFFSET, 0), 
    (0.8, 2.45, 0), 
    (3.91, 1.75 - CAR_FRONT_OFFSET, pi/2)
]

START = PLANNING_WAYPOINTS[0]
END = PLANNING_WAYPOINTS[3]

MAX_PHI = pi/3
DT = 0.005
UNIT_THETA = pi/24

class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid_robplan, reverse, unit_theta=pi/12, dt=1e-2, check_dubins=1):
        self.car = car
        self.grid = grid_robplan
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.drive_steps = int(sqrt(2)*self.grid.cell_size/self.dt) + 1
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        self.dubins = DubinsPath(self.car)
        self.reeds = ReedsSheppPath(self.car)
        self.astar = Astar(self.grid, self.goal[:2])
        
        self.w1 = 0.95 # weight for astar heuristic
        self.w2 = 0.05 # weight for simple heuristic
        self.w3 = 0.30 # weight for extra cost of steering angle change
        self.w4 = 0.10 # weight for extra cost of turning
        self.w5 = 2.00 # weight for extra cost of reversing

        self.thetas = get_discretized_thetas(self.unit_theta)

    def construct_node(self, pos):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        theta = round_theta(theta % (2*pi), self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id + [theta]

        node = Node(grid_pos, pos)

        return node
    
    def simple_heuristic(self, pos):
        """ Heuristic by Manhattan distance. """

        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])
        
    def astar_heuristic(self, pos):
        """ Heuristic by standard astar. """

        h1 = self.astar.search_path(pos[:2])
        if h1 is None:
            h1 = 0.0
        h2 = self.simple_heuristic(pos[:2])
        
        return self.w1*h1*self.grid.cell_size + self.w2*h2

    def get_children(self, node, heu, extra):
        """ Get successors from a state. """

        children = []
        for m, phi in self.comb:

            # don't go back
            if node.m and node.phi == phi and node.m*m == -1:
                continue

            if node.m and node.m == 1 and m == -1:
                continue

            pos = node.pos
            branch = [m, pos[:2]]

            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m)
                branch.append(pos[:2])

            # check safety of route-----------------------
            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                # extra cost for changing steering angle
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                
                # extra cost for turning
                if phi != 0:
                    child.g += self.w4 * self.arc
                
                # extra cost for reverse
                if m == -1:
                    child.g += self.w5 * self.arc

            if heu == 0:
                child.f = child.g + self.simple_heuristic(child.pos)
            if heu == 1:
                child.f = child.g + self.astar_heuristic(child.pos)
            
            children.append([child, branch])

        return children
    
    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        """ Search best final shot in open set. """

        open_.sort(key=lambda x: x.f, reverse=False)

        for t in range(min(n, len(open_))):
            best_ = open_[t]
            d_route_, cost_, valid_ = self.reeds.find_path(best_.pos, self.goal)
            # solutions_ = self.dubins.find_tangents(best_.pos, self.goal)
            # d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
        
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best = best_
                cost = cost_
                d_route = d_route_
        
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        
        return best, cost, d_route
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start)
        root.g = float(0)
        root.g_ = float(0)
        
        if heu == 0:
            root.f = root.g + self.simple_heuristic(root.pos)
        if heu == 1:
            root.f = root.g + self.astar_heuristic(root.pos)

        closed_ = []
        open_ = [root]

        count = 0
        while open_:
            count += 1
            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check dubins path
            if count % self.check_dubins == 0:
                # solutions = self.dubins.find_tangents(best.pos, self.goal)
                # d_route, cost, valid = self.dubins.best_tangent(solutions)
                d_route, cost, valid = self.reeds.find_path(best.pos, self.goal)

                if valid:
                    best, cost, d_route = self.best_final_shot(open_, closed_, best, cost, d_route)
                    route = self.backtracking(best) + d_route
                    path = self.car.get_path(self.start, route)
                    cost += best.g_
                    print('Shortest path: {}'.format(round(cost, 2)))
                    print('Total iteration:', count)
                    
                    return path, closed_

            children = self.get_children(best, heu, extra)

            for child, branch in children:

                if child in closed_:
                    continue

                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)

                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)

                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    
                    open_.remove(child)
                    open_.append(child)

        return None, None

def main_hybrid_a(start_pos, end_pos, visualise=False):
    reverse = True
    grid_on = True
    subsampling_rate = 10

    tc = MapGrid()
    env = Environment(tc.obs, lx=ENV_LENGTH, ly=ENV_WIDTH)
    car = SimpleCar(env, start_pos, end_pos, l=CAR_L, max_phi=MAX_PHI)
    grid = Grid(env, cell_size=GRID_SIZE)

    hastar = HybridAstar(car, grid, reverse, unit_theta=UNIT_THETA, dt=DT)

    valid_start_end = car.is_pos_safe(start_pos) and car.is_pos_safe(end_pos)
    if not valid_start_end:
        print("Start and/or end position is not valid")
        print("Bounding for start: ", car.get_car_bounding(start_pos))
        print("Bounding for end: ", car.get_car_bounding(end_pos))
        print("Start is safe? ", car.is_pos_safe(start_pos))
        print("End is safe? ", car.is_pos_safe(end_pos))

    t = time.time()
    if valid_start_end:
        path, closed_ = hastar.search_path()
    else:
        path = None
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    xl, yl = [], []
    xl_np1,yl_np1=[],[]

    if not path:
        print('No valid path!')
    else:
        # path = path[::subsampling_rate] + [path[-1]]
        #for i in range(len(path)):
        #    print(path[i].pos[0])
        
        branches = []
        bcolors = []
        for node in closed_:
            for b in node.branches:
                branches.append(b[1:])
                bcolors.append('y' if b[0] == 1 else 'b')

        carl = []
        dt_s=int(subsampling_rate)  # samples for gazebo simulator
        for i in range(len(path)):
            xl.append(path[i].pos[0])
            yl.append(path[i].pos[1])
            carl.append(path[i].model[0])
            if i==0 or i==len(path):
                xl_np1.append(path[i].pos[0])
                yl_np1.append(path[i].pos[1])            
            elif dt_s*i<len(path):
                xl_np1.append(path[i*dt_s].pos[0])
                yl_np1.append(path[i*dt_s].pos[1])
    
    if visualise:
        start_state = car.get_car_state(car.start_pos)
        end_state = car.get_car_state(car.end_pos)

        # plot and annimation
        fig, ax = plt.subplots(figsize=(6,6))
        ax.set_xlim(0, env.lx)
        ax.set_ylim(0, env.ly)
        ax.set_aspect("equal")

        if grid_on:
            ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
            ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            ax.tick_params(length=0)
            plt.grid(which='both')
        else:
            ax.set_xticks([])
            ax.set_yticks([])
        
        for ob in env.obs:
            ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
        
        ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
        ax = plot_a_car(ax, end_state.model)
        ax = plot_a_car(ax, start_state.model)

        if path:
            _branches = LineCollection([], linewidth=1)
            ax.add_collection(_branches)

            _path, = ax.plot([], [], color='lime', linewidth=2)
            _carl = PatchCollection([])
            ax.add_collection(_carl)
            _path1, = ax.plot([], [], color='w', linewidth=2)
            _car = PatchCollection([])
            ax.add_collection(_car)
            
            frames = len(branches) + len(path) + 1

        def init():
            _branches.set_paths([])
            _path.set_data([], [])
            _carl.set_paths([])
            _path1.set_data([], [])
            _car.set_paths([])

            return _branches, _path, _carl, _path1, _car

        def animate(i):

            edgecolor = ['k']*5 + ['r']
            facecolor = ['y'] + ['k']*4 + ['r']

            if i < len(branches):
                _branches.set_paths(branches[:i+1])
                _branches.set_color(bcolors)
            
            else:
                _branches.set_paths(branches)

                j = i - len(branches)

                _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

                sub_carl = carl[:min(j+1, len(path))]
                _carl.set_paths(sub_carl[::4])
                _carl.set_edgecolor('k')
                _carl.set_facecolor('m')
                _carl.set_alpha(0.1)
                _carl.set_zorder(3)

                _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
                _path1.set_zorder(3)

                _car.set_paths(path[min(j, len(path)-1)].model)
                _car.set_edgecolor(edgecolor)
                _car.set_facecolor(facecolor)
                _car.set_zorder(3)

            return _branches, _path, _carl, _path1, _car

        if path:
            _ = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                    interval=10, repeat=True, blit=True)

        plt.show()
    
    if path:
        return list(zip(xl_np1, yl_np1))
    else:
        return None

class MapGrid:
    def __init__(self):
        self.obs = OBSTACLES

if __name__ == '__main__':
    print("Executing hybrid A* algorithm")
    start_pos = START
    end_pos = END
    main_hybrid_a(start_pos, end_pos,True)
