import imageio.v2 as imageio
import numpy as np
import os
from environment import GridEnvironment
from agent import RRTStarPlanner
import matplotlib.pyplot as plt

def main():
    grid_size = (20, 20)
    start = (3, 16)
    goal = (17, 2)
    # More complex obstacles: multiple walls, blocks, and a maze-like structure
    obstacles = []
    # Vertical walls
    for i in range(3, 17):
        obstacles.append((i, 5))
        obstacles.append((i, 14))
    # Horizontal walls
    for j in range(2, 18):
        obstacles.append((7, j))
        obstacles.append((12, j))
    # Block clusters
    for i in range(2, 5):
        for j in range(2, 5):
            obstacles.append((i, j))
    for i in range(15, 18):
        for j in range(15, 18):
            obstacles.append((i, j))
    # Maze-like zigzag
    for i in range(8, 12):
        if i % 2 == 0:
            for j in range(6, 14):
                obstacles.append((i, j))
        else:
            for j in range(6, 10):
                obstacles.append((i, j))
    env = GridEnvironment(grid_size, start, goal, obstacles)
    planner = RRTStarPlanner(grid_size, start, goal, obstacles, max_iter=2000)
    frames = []
    path = None
    for i in range(1, planner.max_iter + 1):
        if path is None:
            path = planner.plan(iterations=1)
        fig, ax = env.render(path=path, return_fig_ax=True, fancy=True, drone_pos=path[-1] if path else start)
        # Draw tree with a more visible color
        for node in planner.nodes:
            if node.parent:
                ax.plot([node.pos[1], node.parent.pos[1]], [node.pos[0], node.parent.pos[0]], color='#00BFFF', alpha=0.25, linewidth=1)
        if path:
            y, x = zip(*path)
            ax.plot(x, y, 'm.-', linewidth=3, label='Path')
        frame_path = f"frame_{i}.png"
        fig.savefig(frame_path)
        frames.append(frame_path)
        plt.close(fig)
        if path:
            # Save extra frames at the end for a pause on the final result
            for _ in range(15):
                frames.append(frame_path)
            break
    images = [imageio.imread(f) for f in frames]
    imageio.mimsave('simulation.gif', images, duration=0.1)
    imageio.mimsave('simulation.mp4', images, fps=10)
    for f in set(frames):
        if os.path.exists(f):
            os.remove(f)
    print("Simulation video and gif saved as simulation.gif and simulation.mp4")
    env.render(path=path, fancy=True)

if __name__ == "__main__":
    main()
