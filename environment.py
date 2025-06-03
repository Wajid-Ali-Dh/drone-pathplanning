import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class GridEnvironment:
    def __init__(self, grid_size=(10, 10), start=(0, 0), goal=(9, 9), obstacles=None):
        self.grid_size = grid_size
        self.start = start
        self.goal = goal
        self.obstacles = obstacles if obstacles else []
        self.reset()

    def reset(self):
        self.agent_pos = self.start
        return self.agent_pos

    def step(self, action):
        # Not used in RRT*
        pass

    def render(self, path=None, save_path=None, return_fig_ax=False, fancy=False, drone_pos=None):
        fig, ax = plt.subplots(figsize=(7, 7))
        # Fancy background
        if fancy:
            ax.set_facecolor('#22223b')
        ax.set_xlim(-0.5, self.grid_size[0] - 0.5)
        ax.set_ylim(-0.5, self.grid_size[1] - 0.5)
        ax.set_xticks(np.arange(-0.5, self.grid_size[0], 1))
        ax.set_yticks(np.arange(-0.5, self.grid_size[1], 1))
        ax.grid(True, color='#c9ada7' if fancy else 'k', linestyle='--', linewidth=0.7, alpha=0.5 if fancy else 1)
        # Draw obstacles
        for obs in self.obstacles:
            rect = patches.Rectangle((obs[1] - 0.5, obs[0] - 0.5), 1, 1, linewidth=0, edgecolor=None, facecolor='#9a031e' if fancy else 'gray', alpha=0.95)
            ax.add_patch(rect)
        # Draw start and goal
        ax.plot(self.start[1], self.start[0], marker='o', markersize=18, color='#43aa8b' if fancy else 'g', label='Start', markeredgewidth=2, markeredgecolor='white')
        ax.plot(self.goal[1], self.goal[0], marker='*', markersize=22, color='#f9c74f' if fancy else 'r', label='Goal', markeredgewidth=2, markeredgecolor='white')
        # Draw path
        if path:
            y, x = zip(*path)
            ax.plot(x, y, 'm.-', linewidth=3, label='Path')
        # Draw drone as a triangle (simple shape)
        if drone_pos is not None:
            tri = patches.RegularPolygon((drone_pos[1], drone_pos[0]), numVertices=3, radius=0.4, orientation=np.pi/2, color='#00b4d8', ec='white', lw=2, zorder=10)
            ax.add_patch(tri)
        ax.legend(loc='upper left', fontsize=12, facecolor='#f2e9e4' if fancy else 'white')
        plt.gca().invert_yaxis()
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path)
            plt.close()
        elif return_fig_ax:
            return fig, ax
        else:
            plt.show()
