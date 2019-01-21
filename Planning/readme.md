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

<p align="center"> 
<img src="images/voxel_map.png" width="250" height="250">
</p>
<p align="center">
 <small>[image source](https://www.google.com/url?sa=i&source=images&cd=&cad=rja&uact=8&ved=2ahUKEwjT9vDzw__fAhXNoFsKHWD2BZEQjRx6BAgBEAU&url=https%3A%2F%2Fwww.researchgate.net%2Ffigure%2Fa-Dense-maximum-likelihood-occupancy-voxel-map-of-the-environment-depicted-in-Fig-5a_fig5_228941748&psig=AOvVaw0zbwcyghRHlWpZxT7ixI-i&ust=1548182622594675)
 </small>
 </p>
 
 
 
