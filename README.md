# 🚁 Drone Path Planning and Dynamic Obstacle Avoidance using RRT*

This project simulates autonomous drone navigation in a 2D environment using the RRT* (Rapidly-exploring Random Tree Star) algorithm — a powerful method for optimal path planning in dynamic, cluttered spaces.
The simulation shows a drone starting from a challenging location and intelligently navigating toward a distant goal while avoiding complex, maze-like obstacles.

## ✨ Key Features
- **RRT* Algorithm**: Efficiently finds an optimal path from start to goal while avoiding obstacles.
- **🔄 Optimal Path Planning with RRT***: Finds efficient paths while continuously refining them as the tree expands.
- **🧱 Challenging Environments**: Includes clusters of obstacles, walls, and maze structures to test real-world feasibility.
- **🚁 Beautiful Simulation**: The environment is rendered with a modern, visually appealing style.
- **🎞 Full Visualization**: Watch the entire planning process unfold — from tree growth to path execution — in a rendered GIF
## Simulation Example

![Simulation](simulation.gif)

## How to Run
1. Install dependencies:
   ```bash
   pip install numpy matplotlib imageio
   ```
2. Run the simulation:
   ```bash
   python3 main.py
   ```
3. View the generated `simulation.gif` and `simulation.mp4` for the full planning process.

## Files
- `main.py`: Main entry point, runs the simulation and saves the animation.
- `environment.py`: Defines the grid, obstacles, rendering, and drone visualization.
- `agent.py`: Implements the RRT* path planning algorithm.
- `simulation.gif`/`simulation.mp4`: Animation of the planning process.
- `utils.py`: Utility functions (optional).

---

Feel free to use, modify, or extend this project for your own drone path planning and robotics research!
