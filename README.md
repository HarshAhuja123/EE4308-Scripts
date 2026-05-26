# EE4308-Scripts

This project covers the implementation of a Regulated Pure Pursuit Control with distance-based velocity control heuristics, as well as a standard A* path planner algorithm with tuned Savitzky Golay Smoothing. 

The Regulated Pure Pursuit Controller contains the following features
1. Distance Heuristic: if the robot's global pose is within a specific proximity with respect to a high cost area, with said specific proximity being a tunable real minimum distance, the robot's velocity is scaled by the ratio between the distance limit and computed distance.
2. Curvature Heuristic: to improve the robot's path-following behaviour specifically, if the 
