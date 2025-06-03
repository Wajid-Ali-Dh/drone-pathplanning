import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random

class Node:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent
        self.cost = 0

class RRTStarPlanner:
    def __init__(self, grid_size, start, goal, obstacles, max_iter=500, step_size=1, goal_sample_rate=0.1, search_radius=2):
        self.grid_size = grid_size
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = set(obstacles)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.nodes = [self.start]

    def plan(self, iterations=None):
        iters = iterations if iterations is not None else self.max_iter
        for i in range(iters):
            rnd = self.sample()
            nearest = self.get_nearest_node(rnd)
            new_pos = self.steer(nearest.pos, rnd)
            if not self.is_collision(nearest.pos, new_pos):
                new_node = Node(new_pos, nearest)
                new_node.cost = nearest.cost + self.distance(nearest.pos, new_pos)
                near_nodes = self.find_near_nodes(new_node)
                min_cost = new_node.cost
                min_node = nearest
                for near in near_nodes:
                    if not self.is_collision(near.pos, new_node.pos):
                        cost = near.cost + self.distance(near.pos, new_node.pos)
                        if cost < min_cost:
                            min_cost = cost
                            min_node = near
                new_node.parent = min_node
                new_node.cost = min_cost
                self.nodes.append(new_node)
                self.rewire(new_node, near_nodes)
                if self.distance(new_node.pos, self.goal.pos) <= self.step_size:
                    if not self.is_collision(new_node.pos, self.goal.pos):
                        self.goal.parent = new_node
                        self.goal.cost = new_node.cost + self.distance(new_node.pos, self.goal.pos)
                        self.nodes.append(self.goal)
                        return self.extract_path()
        return None

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.pos
        return (random.randint(0, self.grid_size[0]-1), random.randint(0, self.grid_size[1]-1))

    def get_nearest_node(self, pos):
        return min(self.nodes, key=lambda node: self.distance(node.pos, pos))

    def steer(self, from_pos, to_pos):
        x1, y1 = from_pos
        x2, y2 = to_pos
        dx = x2 - x1
        dy = y2 - y1
        dist = np.hypot(dx, dy)
        if dist == 0:
            return from_pos
        scale = min(self.step_size, dist) / dist
        x = int(round(x1 + dx * scale))
        y = int(round(y1 + dy * scale))
        x = max(0, min(self.grid_size[0]-1, x))
        y = max(0, min(self.grid_size[1]-1, y))
        return (x, y)

    def is_collision(self, from_pos, to_pos):
        x1, y1 = from_pos
        x2, y2 = to_pos
        points = self.bresenham(x1, y1, x2, y2)
        for p in points:
            if p in self.obstacles:
                return True
        return False

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def distance(self, p1, p2):
        return np.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def find_near_nodes(self, new_node):
        n = len(self.nodes)
        r = min(self.search_radius, np.sqrt((np.log(n) / n)))
        return [node for node in self.nodes if self.distance(node.pos, new_node.pos) <= r]

    def rewire(self, new_node, near_nodes):
        for near in near_nodes:
            if not self.is_collision(new_node.pos, near.pos):
                cost = new_node.cost + self.distance(new_node.pos, near.pos)
                if cost < near.cost:
                    near.parent = new_node
                    near.cost = cost

    def extract_path(self):
        path = []
        node = self.goal
        while node is not None:
            path.append(node.pos)
            node = node.parent
        return path[::-1]

class QLearningAgent:
    def __init__(self, state_shape, n_actions, alpha=0.1, gamma=0.99, epsilon=1.0, epsilon_decay=0.995, epsilon_min=0.05):
        self.q_table = np.zeros(state_shape + (n_actions,))
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.n_actions = n_actions

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.randint(self.n_actions)
        return np.argmax(self.q_table[state])

    def learn(self, state, action, reward, next_state, done):
        best_next = np.max(self.q_table[next_state])
        td_target = reward + self.gamma * best_next * (not done)
        td_error = td_target - self.q_table[state + (action,)]
        self.q_table[state + (action,)] += self.alpha * td_error
        if done:
            self.epsilon = max(self.epsilon * self.epsilon_decay, self.epsilon_min)
