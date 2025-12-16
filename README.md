# An-Adaptive-Path-Planning-Framework-for-Indoor-Robots-using-Sensor-Fusion-Phase-1

## Description
This repository contains the MATLAB code for Phase-1 simulation of an indoor robot navigation system. The system combines **RRT\*** global path planning with **sensor-assisted local navigation** using LIDAR and reactive control. Runtime metrics like collision count, replanning latency, path length, and success rate are logged.

## Requirements
- MATLAB 2024b
- Image Processing Toolbox (for floor-plan preprocessing)
- Basic familiarity with MATLAB plotting and `ginput`

## Instructions to Run
1. Open MATLAB and navigate to the project folder.
2. Run the main script by typing:
    ```matlab
    final_code_v2
    ```
3. When prompted, select a floor-plan image (PNG, JPG, BMP).
4. Click on the figure to select **start** and **goal** points.
5. The simulation will begin, showing:
    - Planned RRT* path (blue)
    - Robot trajectory (red)
    - LIDAR points (yellow)
6. Metrics will be automatically saved as `phase1_metrics.csv`.
7. A video of the simulation will be saved as `phase1_demo_final_v2.mp4`.

## Notes
- The robot performs reactive obstacle avoidance if LIDAR detects nearby obstacles.
- Path smoothing is applied using Gaussian filtering for smoother trajectories.
- Replanning is triggered dynamically when obstacles block the planned path.
