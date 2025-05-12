# rrt_grid_demo.py
# PEP 8 compliant (≤ 79 char/line)

import numpy as np
import matplotlib.pyplot as plt
import heapq
import random

# --------------------------------------------------------------------------- #
# basic data structures                                                       #
# --------------------------------------------------------------------------- #
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

    def __lt__(self, other):
        return self.cost < other.cost


# --------------------------------------------------------------------------- #
# utility functions                                                           #
# --------------------------------------------------------------------------- #
def distance(a, b):
    return np.hypot(a.x - b.x, a.y - b.y)


def steer(src, dst, step=0.1):
    d = distance(src, dst)
    if d <= step:
        return Node(dst.x, dst.y)
    th = np.arctan2(dst.y - src.y, dst.x - src.x)
    return Node(src.x + step * np.cos(th), src.y + step * np.sin(th))


def is_in_obstacle(n, obstacles):
    return any(distance(n, Node(x, y)) <= r for (x, y, r) in obstacles)


def is_collision_free(a, b, obstacles, step=0.005):
    d = distance(a, b)
    k = max(int(d / step), 1)
    for i in range(k + 1):
        x = a.x + (b.x - a.x) * i / k
        y = a.y + (b.y - a.y) * i / k
        if is_in_obstacle(Node(x, y), obstacles):
            return False
    return True


# --------------------------------------------------------------------------- #
# RRT                                                                         #
# --------------------------------------------------------------------------- #
def rrt(start, goal, obstacles, max_iters=10000, step=0.1):
    nodes = [start]
    for _ in range(max_iters):
        rnd = Node(random.random(), random.random())
        nearest = min(nodes, key=lambda n: distance(n, rnd))
        new = steer(nearest, rnd, step)
        if not is_collision_free(nearest, new, obstacles):
            continue
        new.parent = nearest
        nodes.append(new)
        if distance(new, goal) <= step and is_collision_free(new, goal,
                                                             obstacles):
            goal.parent = new
            nodes.append(goal)
            print("RRT  : goal reached")
            return nodes, goal
    return nodes, None


# --------------------------------------------------------------------------- #
# RRT*                                                                        #
# --------------------------------------------------------------------------- #
def rrt_star(start, goal, obstacles, max_iters=10000, step=0.1,
             radius=0.1, max_nodes=500, continue_after_goal=True):
    nodes = [start]
    best_goal = None
    for _ in range(max_iters):
        rnd = Node(random.random(), random.random())
        nearest = min(nodes, key=lambda n: distance(n, rnd))
        new = steer(nearest, rnd, step)
        if not is_collision_free(nearest, new, obstacles):
            continue

        near = [n for n in nodes
                if distance(n, new) <= radius and
                is_collision_free(n, new, obstacles)]
        parent = nearest
        best = nearest.cost + distance(nearest, new)
        for n in near:
            c = n.cost + distance(n, new)
            if c < best:
                parent, best = n, c
        new.parent, new.cost = parent, best
        nodes.append(new)

        for n in near:
            c = new.cost + distance(new, n)
            if c < n.cost and is_collision_free(new, n, obstacles):
                n.parent, n.cost = new, c

        if distance(new, goal) <= step and is_collision_free(new, goal,
                                                             obstacles):
            g = Node(goal.x, goal.y)
            g.parent = new
            g.cost = new.cost + distance(new, g)
            if best_goal is None or g.cost < best_goal.cost:
                best_goal = g
                nodes.append(g)
            if not continue_after_goal:
                print("RRT*: goal found")
                return nodes, best_goal

        if len(nodes) >= max_nodes:
            print(f"RRT*: max nodes reached ({max_nodes})")
            break

    print("RRT*: goal refined" if best_goal else "RRT*: no path")
    return nodes, best_goal


# --------------------------------------------------------------------------- #
# grid-based A*                                                               #
# --------------------------------------------------------------------------- #
def a_star_grid(start, goal, obstacles, h=0.05):
    w = int(1 / h)
    s_idx = (int(start.x / h), int(start.y / h))
    g_idx = (int(goal.x / h), int(goal.y / h))

    open_set = []
    heapq.heappush(open_set, (0, s_idx))
    came, g_score = {}, {s_idx: 0}
    f_score = {s_idx: distance(start, goal)}

    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (1, 1), (-1, 1), (1, -1)]

    def ok(idx):
        x, y = idx[0] * h, idx[1] * h
        return 0 <= x <= 1 and 0 <= y <= 1 and \
            not is_in_obstacle(Node(x, y), obstacles)

    while open_set:
        cur = heapq.heappop(open_set)[1]
        if cur == g_idx:
            p, c = [], cur
            while c in came:
                p.append(Node(c[0] * h, c[1] * h))
                c = came[c]
            p.append(Node(start.x, start.y))
            return p[::-1]

        for dx, dy in dirs:
            nb = (cur[0] + dx, cur[1] + dy)
            if not ok(nb):
                continue
            tg = g_score[cur] + distance(Node(cur[0] * h, cur[1] * h),
                                         Node(nb[0] * h, nb[1] * h))
            if nb not in g_score or tg < g_score[nb]:
                came[nb] = cur
                g_score[nb] = tg
                f_score[nb] = tg + distance(Node(nb[0] * h, nb[1] * h), goal)
                heapq.heappush(open_set, (f_score[nb], nb))
    return None


# --------------------------------------------------------------------------- #
# helper                                                                      #
# --------------------------------------------------------------------------- #
def extract_path(n):
    path = []
    while n:
        path.append(n)
        n = n.parent
    return path[::-1]


# --------------------------------------------------------------------------- #
# plotting                                                                    #
# --------------------------------------------------------------------------- #
def plot_path(nodes, path, start, goal, obstacles, title):
    plt.figure()
    ax = plt.gca()
    plt.grid(False)

    for x, y, r in obstacles:
        ax.add_patch(plt.Circle((x, y), r, color='gray', fill=True, zorder=2))

    for n in nodes:
        if n.parent:
            plt.plot([n.x, n.parent.x], [n.y, n.parent.y],
                     'b-', linewidth=0.5)

    for i in range(len(path) - 1):
        plt.plot([path[i].x, path[i + 1].x],
                 [path[i].y, path[i + 1].y],
                 'r-', linewidth=2)

    plt.scatter(start.x, start.y, c='green', s=100, label="Start", zorder=10)
    plt.scatter(goal.x, goal.y, c='red', s=100, label="Goal", zorder=10)

    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.title(title)

    lg = ax.legend(loc='lower right', fancybox=True, shadow=True, frameon=True)
    lg.set_zorder(20)

    plt.tight_layout()
    plt.show()


def plot_grid_path(grid_path, start, goal, obstacles, h,
                   title, grid_lw=1.0, grid_a=0.7):
    plt.figure()
    ax = plt.gca()
    plt.grid(False)

    ticks = np.arange(0, 1 + h, h)
    for x in ticks:
        for y0, y1 in zip(ticks[:-1], ticks[1:]):
            if not is_in_obstacle(Node(x, (y0 + y1) / 2), obstacles):
                plt.plot([x, x], [y0, y1], color='#1f77b4',
                         linewidth=grid_lw, alpha=grid_a)
    for y in ticks:
        for x0, x1 in zip(ticks[:-1], ticks[1:]):
            if not is_in_obstacle(Node((x0 + x1) / 2, y), obstacles):
                plt.plot([x0, x1], [y, y], color='#1f77b4',
                         linewidth=grid_lw, alpha=grid_a)

    for x, y, r in obstacles:
        ax.add_patch(plt.Circle((x, y), r, color='gray', fill=True, zorder=2))

    for i in range(len(grid_path) - 1):
        a, b = grid_path[i], grid_path[i + 1]
        if not is_in_obstacle(a, obstacles) and not is_in_obstacle(b,
                                                                  obstacles):
            plt.plot([a.x, b.x], [a.y, b.y], 'r-', linewidth=2)

    plt.scatter(start.x, start.y, c='green', s=100, label="Start", zorder=10)
    plt.scatter(goal.x, goal.y, c='red', s=100, label="Goal", zorder=10)

    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.title(title)

    lg = ax.legend(loc='lower right', fancybox=True, shadow=True, frameon=True)
    lg.set_zorder(20)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

def plot_initial(start, goal, obstacles, title):
    plt.figure()
    ax = plt.gca()
    plt.grid(False)

    for x, y, r in obstacles:
        ax.add_patch(plt.Circle((x, y), r, color='gray', fill=True, zorder=2))

    plt.scatter(start.x, start.y, c='green', s=100, label="Start", zorder=10)
    plt.scatter(goal.x, goal.y, c='red', s=100, label="Goal", zorder=10)

    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.title(title)

    lg = ax.legend(loc='lower right', fancybox=True, shadow=True, frameon=True)
    lg.set_zorder(20)

    plt.tight_layout()
    plt.show()

# --------------------------------------------------------------------------- #
# main demo                                                                   #
# --------------------------------------------------------------------------- #
if __name__ == "__main__":


    start = Node(0.1, 0.1)
    goal = Node(0.9, 0.9)
    obs = [(0.2, 0.8, 0.1),
           (0.4, 0.45, 0.25),
           (0.8, 0.2, 0.2)]

    step = 0.04

    # plain RRT
    tree, g = rrt(Node(start.x, start.y), Node(goal.x, goal.y),
                  obs, step=step)
    plot_initial(start, goal, obs, "")
    if g:
        plot_path(tree, extract_path(g), start, goal, obs,
                  f"RRT (step size={step} | number of nodes={len(tree)})")

    # RRT*
    tree_s, g_s = rrt_star(Node(start.x, start.y), Node(goal.x, goal.y),
                           obs, step=step, radius=0.1,
                           max_nodes=700, continue_after_goal=False)
    if g_s:
        plot_path(tree_s, extract_path(g_s), start, goal, obs,
                  f"RRT* (step size={step} | number of nodes={len(tree_s)})")

    tree_s, g_s = rrt_star(Node(start.x, start.y), Node(goal.x, goal.y),
                        obs, step=step, radius=0.1,
                        max_nodes=676, continue_after_goal=True)
    if g_s:
        plot_path(tree_s, extract_path(g_s), start, goal, obs,
                  f"RRT* (step size={step} | number of nodes={len(tree_s)})")
    # grid A*
    grid = a_star_grid(Node(start.x, start.y), Node(goal.x, goal.y),
                       obs, h=step)
    if grid:
        ticks = np.arange(0, 1 + step, step)
        tot = len(ticks) * len(ticks)
        title_grid = (f"A* Grid-Search (grid size={step:.2f} | "
                      f"grid points={tot})")
        plot_grid_path(grid, start, goal, obs, h=step,
                       title=title_grid,
                       grid_lw=1.0, grid_a=0.7)
