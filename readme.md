## <p align="center"> <b> Planning </b> </p>

Moving a vehicle from goal to destination is a fundamental problem in autonomous vehicles. There are numerous approaches with new methodologies appearing frequently, especially with the advent of machine learning. This notebook demonstrates how a grid-based global planner paired with a local planner based on random sampling is able to navigate a drone through a simulation of San Francisco. The end of the readme adapts these same principles to a real drone and successfully navigates the drone through a tree line. 

## <p align="center"> <b> Control </b> </p>

Quadcopter's exist in 3-dimensions with 6 degrees of freedom: forward/back, up/down, left/right, and rotations about each axis. The physics of a quadcopter are very well understood, and as such, sophisticated controllers can be constructed to dictate a quad's movement. The controller implemented in this directory is a Cascaded PID Controller and is very similar to approaches taken by Ardupilot and PX4.

## <p align="center"> <b> Estimation </b> </p>

finishing up soon...
