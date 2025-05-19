#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import random
from math import pi
import math
import argparse
from matplotlib.patches import Rectangle
from scipy import interpolate
from scipy.interpolate import CubicSpline
import rospy



OBSTACLES = [
    (1.2, 1.65, 0.2, 0.4),
    (2.2, 1.65, 0.4, 0.4),
    (1.35, 0.6, 0.5, 0.2),
    (3.06, 0.7, 0.5, 0.2),
    (3.71, 1.75, 0.4, 0.2)
]

WALLS = [
    (0.0, 1.0, 0.5, 0.2),
    (3.31, 0, 1.9, 0.2)
]

PLANNING_WAYPOINTS = [
    (0.2, 0.1, 0), 
    (1.6, 0.6, 0), 
    (3.31, 0.9, -pi/2), 
    (3.21, 2.75, pi/2), 
    (5.21, 0.3, 0), 
    (0.8, 2.45, 0), 
    (3.91, 1.75, pi/2)
]


REAL_OBSTACLES = [
    (1.2, 1.65, 0.2, 0.4),
    (2.3, 1.65, 0.4, 0.4),
    (1.45, 0.7, 0.5, 0.2),
    (3.16, 0.7, 0.5, 0.2),
    (3.61, 1.75, 0.4, 0.2)
]

REAL_WALLS = [
    (0.0, 1.0, 0.5, 0.2),
    (3.3, 0, 1.91, 0.2)
]

REAL_PLANNING_WAYPOINTS = [
    (0.35, 0.35, 0), 
    (1.75, 0.7, 0), 
    (3.45, 0.9, -pi/2), 
    (3.31, 2.75, pi/2), 
    (5.21, 0.2, 0), 
    (0.95, 2.55, 0), #5
    (3.85, 1.75, pi/2) #6
]





class RRT_node:
    """ RRT tree node """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0


class RRTStar:
    def __init__(self, start, goal, map_size, step_size=0.1, max_iter=1000, radius_to_goal=0.05, real_map=True):
        self.start = RRT_node(start[0], start[1])
        self.goal = RRT_node(goal[0], goal[1])
        self.map_size = map_size
        if real_map:
            self.obstacles = REAL_OBSTACLES
            self.walls = REAL_WALLS
        else:
            self.obstacles = OBSTACLES
            self.walls = WALLS
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.goal_region_radius = radius_to_goal
        self.search_radius = 0.3
        self.path = None
        self.goal_reached = False
        self.padding = 0.275
        self.wall_padding = 0.25
        self.start = self.adjustPos(self.start)
        self.goal = self.adjustPos(self.goal)

        # Visualization setup
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.map_visualization()


    
    def map_visualization(self):
        """Set up the visualization environment (grid, start, goal, obstacles)."""
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.set_aspect("equal")
        self.ax.plot(self.start.x, self.start.y, 'bo', label='Start')
        self.ax.plot(self.goal.x, self.goal.y, 'ro', label='Goal')
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.grid(True)

        # Draw the obstacles
        self.draw_obstacles()

    def draw_obstacles(self):
        """Draw the static obstacles with padding and wall padding on the map."""
        self.ax.add_patch(Rectangle((0,0), self.wall_padding, self.map_size[1], fc=(204/255, 205/255, 207/255)))
        self.ax.add_patch(Rectangle((0,0), self.map_size[0], self.wall_padding, fc=(204/255, 205/255, 207/255)))
        self.ax.add_patch(Rectangle((0,self.map_size[1]-self.wall_padding), self.map_size[0], self.wall_padding, fc=(204/255, 205/255, 207/255)))
        self.ax.add_patch(Rectangle((self.map_size[0]-self.wall_padding,0), self.wall_padding, self.map_size[1], fc=(204/255, 205/255, 207/255)))
        for (ox, oy, w, h) in self.obstacles:
            self.ax.add_patch(Rectangle((ox-self.padding, oy-self.padding), w+2*self.padding, h+2*self.padding, fc=(204/255, 205/255, 207/255), ec='k'))
            self.ax.add_patch(Rectangle((ox, oy), w, h, fc='gray', ec='k'))
        for (ox, oy, w, h) in self.walls:
            self.ax.add_patch(Rectangle((ox-self.wall_padding, oy-self.wall_padding), w+2*self.wall_padding, h+2*self.wall_padding, fc=(204/255, 205/255, 207/255)))
            self.ax.add_patch(Rectangle((ox, oy), w, h, fc='gray'))
        

    def plan(self):
        """Main RRT* planning loop."""
        for i in range(self.max_iter):
            if rospy.is_shutdown():
                break
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if self.is_collision_free(new_node) and self.is_not_in_wall(new_node):
                neighbors = self.find_neighbors(new_node)
                new_node = self.choose_parent(neighbors, nearest_node, new_node)
                self.node_list.append(new_node)
                self.rewire(new_node, neighbors)

        goal_node = self.get_goal_node(self.goal)
        if goal_node:
            self.path = self.generate_final_path(goal_node)
            self.goal_reached = True
        return    
        

    def get_random_node(self):
        """Generate a random node in the map."""
        if random.random() > 0.2:
            rand_node = RRT_node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            rand_node = RRT_node(self.goal.x, self.goal.y)
        return rand_node

    def steer(self, from_node, to_node):
        """Steer from one node to another, step-by-step."""
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = RRT_node(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        return new_node

    def adjustPos(self, node):
        """Adjusts a position to be outside the padding of obstacles"""
        if not self.is_collision_free(node):
            # Find the respective obstacle
            x = node.x
            y = node.y
            for obs in self.obstacles:
                if (x > obs[0]-self.padding) and (x < obs[0]+obs[2]+self.padding) and (y > obs[1]-self.padding) and (y < obs[1]+obs[3]+self.padding):
                    new_pos_l = obs[0]-self.padding, y
                    new_pos_r = obs[0]+obs[2]+self.padding, y
                    new_pos_d = x, obs[1]-self.padding
                    new_pos_u = x, obs[1]+obs[3]+self.padding

                    pos = x, y

                    dist_l = self.distance(pos, new_pos_l)
                    dist_r = self.distance(pos, new_pos_r)
                    dist_d = self.distance(pos, new_pos_d)
                    dist_u = self.distance(pos, new_pos_u)

                    distances = list()
                    distances.append(dist_l)
                    distances.append(dist_r)
                    distances.append(dist_d)
                    distances.append(dist_u)
                    
                    positions = list()
                    positions.append(new_pos_l)
                    positions.append(new_pos_r)
                    positions.append(new_pos_d)
                    positions.append(new_pos_u)

                    new_pos = pos

                    done = False

                    while not done:
                        idx = distances.index(min(distances))
                        new_pos = positions[idx]
                        new_node = RRT_node(x=new_pos[0], y=new_pos[1])

                        if self.is_collision_free(new_node):
                            done = True
                            node.x = new_pos[0]
                            node.y = new_pos[1]
                            return self.move_out_of_wall(node)

                        distances.pop(idx)
                        positions.pop(idx)

        return self.move_out_of_wall(node)
    
    def move_out_of_wall(self, node):
        """Moves a node to the closest point outside of the wall padding"""
        if node.x < self.wall_padding:
            node.x = self.wall_padding
        elif node.x > self.map_size[0] - self.wall_padding:
            node.x = self.map_size[0] - self.wall_padding

        if node.y < self.wall_padding:
            node.y = self.wall_padding
        elif node.y > self.map_size[1] - self.wall_padding:
            node.y = self.map_size[1] - self.wall_padding

        return node

    def distance(self, pos1, pos2):
        """Calculates the distance between two points"""
        x1, y1 = pos1
        x2, y2 = pos2

        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def is_collision_free(self, node):
        """Check if the node is collision-free with respect to obstacles."""
        for obs in self.obstacles:
            if (node.x > obs[0]-self.padding) and (node.x < obs[0]+obs[2]+self.padding) and (node.y > obs[1]-self.padding) and (node.y < obs[1]+obs[3]+self.padding):
                return False
        for wall in self.walls:
            if (node.x > wall[0]-self.wall_padding) and (node.x < wall[0]+wall[2]+self.wall_padding) and (node.y > wall[1]-self.wall_padding) and (node.y < wall[1]+wall[3]+self.wall_padding):
                return False
        return True
            
    def is_not_in_wall(self, node):
        """Check if node is in wall padding"""
        if (node.x < self.wall_padding) or (node.x > self.map_size[0]-self.wall_padding) or (node.y < self.wall_padding) and (node.y > self.map_size[1]-self.wall_padding):
                return False
        return True
    
    def is_not_through_obstacle(self, node1, node2):
        """Checks if parts of the path are within obstacles"""
        # TODO: Optimize to check whether obsatcle borders and path intersect

        num_points = 100

        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y

        dx = (x2-x1)/num_points
        dy = (y2-y1)/num_points

        for i in range(num_points-1):
            node = RRT_node(x=x1+i*dx, y=y1+i*dy)
            if not self.is_collision_free(node):
                return False
        return True

    def find_neighbors(self, new_node):
        """Find nearby nodes within the search radius."""
        return [node for node in self.node_list
                if np.linalg.norm([node.x - new_node.x, node.y - new_node.y]) < self.search_radius]

    def choose_parent(self, neighbors, nearest_node, new_node):
        """Choose the best parent for the new node based on cost."""
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y])
        best_node = nearest_node

        for neighbor in neighbors:
            cost = neighbor.cost + np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y])
            if cost < min_cost and self.is_collision_free(neighbor) and self.is_not_through_obstacle(new_node,neighbor):
                best_node = neighbor
                min_cost = cost

        new_node.cost = min_cost
        new_node.parent = best_node
        return new_node

    def rewire(self, new_node, neighbors):
        """Rewire the tree by checking if any neighbor should adopt the new node as a parent."""
        for neighbor in neighbors:
            cost = new_node.cost + np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y])
            if cost < neighbor.cost and self.is_collision_free(neighbor) and self.is_not_through_obstacle(new_node,neighbor):
                neighbor.parent = new_node
                neighbor.cost = cost

    def reached_goal(self, node):
        """Check if the goal has been reached."""
        return np.linalg.norm([node.x - self.goal.x, node.y - self.goal.y]) < self.goal_region_radius

    def get_goal_node(self, goal_node):
        """Finds closest node to the desired goal node"""
        dist = 100
        idx = None
        i = 0
        for node in self.node_list:
            x1 = node.x
            y1 = node.y
            x2 = goal_node.x
            y2 = goal_node.y
            pos1 = x1, y1
            pos2 = x2, y2
            if (self.distance(pos1, pos2) < dist) and (self.distance(pos1, pos2) < self.goal_region_radius):
                idx = i
            i += 1
        if idx:
            return self.node_list[idx]
        return None

    def generate_final_path(self, goal_node):
        """Generate the final path from the start to the goal."""
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        path[0] = [self.goal.x, self.goal.y]
        return path[::-1]  # Reverse the path

    def get_nearest_node(self, node_list, rand_node):
        """Find the nearest node in the tree to the random node."""
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node = node_list[np.argmin(distances)]
        return nearest_node

    def draw_tree(self, node):
        """Draw a tree edge from the current node to its parent."""
        if node.parent:
            self.ax.plot([node.x, node.parent.x], [node.y, node.parent.y], "-b")

    def draw_path(self):
        """Draw the final path from start to goal."""
        if self.path:
            self.ax.plot([x[0] for x in self.path], [x[1] for x in self.path], '-g', label='Path')
            # plt.show()

    def draw_smooth_path(self, smooth):
        """Draws smooth path from start to goal"""
        for i in range(len(smooth)-1):
            self.ax.plot([smooth[i][0], smooth[i+1][0]], [smooth[i][1], smooth[i+1][1]], '-r')

    def splines(self, path):
        """Creates a b-spline path to smoothen the resulting path from RRT"""
        x = []
        y = []
        for point in path:
            x.append(point[0])
            y.append(point[1])
        tck, *rest = interpolate.splprep([x,y])
        u = np.linspace(0, 1, num=100)
        smooth = interpolate.splev(u, tck)
        plan = []
        for i in range(len(smooth[0])):
            plan.append([smooth[0][i], smooth[1][i]])
        return plan
    
    def cub_splines(self, path, init_epsilon):
        """Creates a cubic-spline path to smoothen the resulting path from RRT"""
        epsilon = init_epsilon
        spline_valid = False
        orig_path = path
        v_mean = 0.15 # TODO  avg speed of the robot, can we do 0.25 here? right now

        while not spline_valid and epsilon > 0:
            path = self.prune_path(orig_path, epsilon)
            x = []
            y = []
            for point in path:
                x.append(point[0])
                y.append(point[1])

            distances = [0]
            cumulative_dist = 0
            seg_epsilon = 1e-10
            path = np.array(path)
            for i in range(1, len(path)):
                segment_length = max(
                    seg_epsilon,
                    np.sqrt(np.sum((path[i] - path[i-1])**2))
                )
                cumulative_dist += segment_length
                distances.append(cumulative_dist)

            T_sim = cumulative_dist / v_mean
            t_waypoints = np.array(distances) / v_mean 
            x_spline = CubicSpline(t_waypoints, x)
            y_spline = CubicSpline(t_waypoints, y)
            u = np.linspace(0, T_sim, num=100)
            spline_valid = self.checkSpline(x_spline, y_spline, T_sim)
            epsilon = epsilon - 0.001
        
        path = orig_path

        if epsilon <= 0 and not spline_valid:
            x = []
            y = []
            for point in path:
                x.append(point[0])
                y.append(point[1])

            distances = [0]
            cumulative_dist = 0
            seg_epsilon = 1e-10
            path = np.array(path)
            for i in range(1, len(path)):
                segment_length = max(
                    seg_epsilon,
                    np.sqrt(np.sum((path[i] - path[i-1])**2))
                )
                cumulative_dist += segment_length
                distances.append(cumulative_dist)
            T_sim = cumulative_dist / v_mean
            t_waypoints = np.array(distances) / v_mean 
            x_spline = CubicSpline(t_waypoints, x)
            y_spline = CubicSpline(t_waypoints, y)

        smooth_x = x_spline(u)
        smooth_y = y_spline(u)
        plan = []
        for i in range(len(smooth_x)):
            plan.append([smooth_x[i], smooth_y[i]])
        return x_spline, y_spline, T_sim, plan
    
    def checkSpline(self, x_spline, y_spline, T_sim):
        t = np.linspace(0.01, 0.99*T_sim, num=100)
        for i in range(len(t)):
            node = RRT_node(x_spline(t[i]), y_spline(t[i]))
            if not self.is_collision_free(node):
                return False
        return True          

    def perpendicular_distance(self, point, line_start, line_end):
        """Calculate the perpendicular distance from a point to a line."""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end

        if (x1, y1) == (x2, y2):
            return math.hypot(x0 - x1, y0 - y1)

        num = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1)
        den = math.hypot(y2 - y1, x2 - x1)
        return num / den

    def prune_path(self, path, epsilon):
        """Prune waypoints from a path using the Ramer-Douglas-Peucker algorithm"""
        if len(path) < 3:
            return path

        # Find the point with the maximum distance from the line
        max_dist = 0.0
        index = 0
        for i in range(1, len(path) - 1):
            dist = self.perpendicular_distance(path[i], path[0], path[-1])
            if dist > max_dist:
                max_dist = dist
                index = i

        # If the max distance is greater than epsilon, recursively simplify
        if max_dist > epsilon:
            left = self.prune_path(path[:index+1], epsilon)
            right = self.prune_path(path[index:], epsilon)

            # Combine results and remove duplicate at the split
            return left[:-1] + right
        else:
            # All points are within epsilon, return just the endpoints
            return [path[0], path[-1]]

    

def main_RRTstar(start_pos, end_pos, max_iter=1000, stepsize=0.1, radius_to_goal=0.1, real_map=True, plot=False): #TODO tune iterations and radius to goal
    start = [start_pos[0], start_pos[1]]
    goal = [end_pos[0], end_pos[1]]
    x_spline, y_spline, T_sim = None, None, None
    map_size = [5.21, 2.75]

    rrt_star = RRTStar(start, goal, map_size, step_size=stepsize, max_iter=max_iter, real_map=real_map, radius_to_goal=radius_to_goal)
    rrt_star.plan()
    if rrt_star.path:
        x_spline, y_spline, T_sim, smooth_path = rrt_star.cub_splines(rrt_star.path, init_epsilon=0.01) #TODO tune epsilon
    else:
        smooth_path = []
    if plot and rrt_star.path:
        rrt_star.draw_path()
        if rrt_star.path:
            rrt_star.draw_smooth_path(smooth_path)
        plt.show()
    return x_spline, y_spline, T_sim, smooth_path


if __name__ == '__main__':
    print("Executing RRT* algorithm")
    p = argparse.ArgumentParser()
    p.add_argument('-heu', type=int, default=1, help='heuristic type')  #A* heuristic
    p.add_argument('-r', action='store_true', help='allow reverse or not')
    p.add_argument('-e', action='store_true', help='add extra cost or not')
    p.add_argument('-g', action='store_true', help='show grid or not')
    args = p.parse_args()
    
    START = REAL_PLANNING_WAYPOINTS[0]
    END = REAL_PLANNING_WAYPOINTS[6]

    start = [START[0], START[1]]
    goal = [END[0], END[1]]
    # start = [3.41, 0.9]
    # goal = [1.7, 0.7]
    map_size = [5.21, 2.75]

    x_spline, y_spline, Tsim, waypoints = main_RRTstar(start, goal, stepsize=0.1, max_iter=1500, radius_to_goal=0.1, real_map=True, plot=True)

    if len(waypoints) < 2:
        print("No solution found!")