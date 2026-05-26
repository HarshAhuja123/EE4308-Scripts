# EE4308-Scripts

This project covers the implementation of a Regulated Pure Pursuit Control with distance-based velocity control heuristics, as well as a standard A* path planner algorithm with tuned Savitzky Golay Smoothing. 

The Regulated Pure Pursuit Controller contains the following features:
1. Distance Heuristic: if the robot's global pose is within a specific proximity with respect to a high cost area, with said specific proximity being a tunable real minimum distance, the robot's velocity is scaled by the ratio between the distance limit and computed distance.
2. Curvature Heuristic: to improve the robot's path-following behaviour specifically, if the curvature of the planned path is greater than a certain value, velocity is scaled down by the ratio of a curvature limit and actual curvature
3. Goal Attainment Thresholds: all goals are defined fully as poses in the x-y plane, with x and y coordinates as well as a yaw value to indicate direction. Since exact attainment of the recorded goal is very difficult and would entail immaculate control, a distance and yaw threshold were set. Particularly, once the robot's pose is within a certain distance from the goal coordinates, the robot is instructed to rotate about its coordinates until the desired yaw value is attained within its own threshold. If the distance threshold for goal attainment is too small, the robot oscillates repetitively about the goal coordinates.

All heuristic and goal-attainment thresholds were tuned in the field, within an actual obstacle course.

The A* Planner and Smoother contain the following features
1. Standard A* Implementation using the Octile Heuristic
2. Post-planning path densificiation to improve path smoothness

It is noteworthy that the path densification process reduces the overall influence of the smoother on path smoothness by reducing the physical coverage of the smoother window. Assuming a fixed path density, a larger smoothing window would create a path with much more gradual curvature changes, but simultaneously reduce obstacle acknowledgement in the path structure. Heightening the polynomial order acts against the intention of smoothing by overfitting the path to the pre-smoothing coordinates. Therefore an ideal smoother configuration comprises a moderate to large window size with low polynomial ordering. 
