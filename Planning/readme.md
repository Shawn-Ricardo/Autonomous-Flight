# <p align="center"><b> Medial-Axis with Probabilistic Roadmap </b></p>
 
Planning is a crucial aspect of any autonomous agent. There exist a vast number of methods that attempt to solve this problem. From physical models that describe the intricate dynamics of the object to the black boxes of reinforcement learning. This problem is still an active area of research. 

Any planning solution will address 5 core areas:

1) State Space 
2) Action Space
3) Cost Function
4) Start State
5) Goal State

A tremendous amount of thought should go into defining your state and action spaces, as these decisions will dictate the hardware that your platform needs to operate in real-time. 

For example, representing the state space by 3-dimensional voxels of resultion 0.05m most likely introduces significant memory utilization and long update times. Requiring powerful processing units, a larger battery, heavier quadcopter, and significantly reduced flight time. 

<p align="center"> <img src="images/voxel_map.png"></p>
<p align="center"><small><i>google images</i></small></p>
 
This approach assumes the the altitude of the quadcopter is fixed as it navigates through a simulation of San Francisco. As such, a 2.5D map is utilized to create a grid state space.

The following is a representation of a 2.5D map,

<p align="center"> <img src="images/two_half.PNG"></p>

In essence, a 2.5D map allows for obstacles to be stored in a 2D grid and the height of that obstacle to be stored as the entry within that cell. Notice that the resolution of obstacles is lost. Planning tight maneuvers around them won't be possible. However, it is usually a bad idea to fly a vehicle very close to an object, such as a building, tree, or car.



Using a grid allows for numerous existing algorithms to be applied to the path planning problem. One in particular is Medial-Axis. Med
 
This approach using a 2-prong planner: 1) a global plan and 2) local planner

### <p align="center"><b><i>Global Plan</i></b></p>
