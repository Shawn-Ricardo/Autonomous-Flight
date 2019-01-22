# <p align="center"><b> Cascaded PID Controller </b></p>

The control of an autonomous flying vehicle boils down to the following objective: Given a trajectory, find the sequence of propeller turn rates that will cause the drone to fly that trajectory. 

A trajectory is a sequence of 3-dimensional positions, desired yaw (heading) and times at which to be in these locations.

In order to place the quadcopter at a desired position with a desired heading, we must control the rotation rates of the propellers. Doing so in such a manner that the net force and net moment acting on the quadcopter will cause it to exhibit the desired position and orientation at the desired timestep.

<p align="center"> <img src="images/quad_forces.PNG"></p>

The illustration above is simplified, only showing forces applied in the horizontal and vertical directions. However, the physics are easily extended when stepping into the 3-dimensional world.

Each propeller generates a thrust vector - a force. To obtain the vertical force applied to the quad, decompose each propellers thrust vector into their corresponding vertical and horizontal components. Sum these forces with the force of gravity and solve for the vertical acceleration. Note: the quad exists in a North-East-Down reference frame, where gravity (straight downward) is positive. Do a similar calculation for horizontal force to find horizontal acceleration. 

Once the vertical and horizontal accelerations are obtained, the translation of the quadrotor can be easily found by integrating twice over some timestep, dt.

<p align="center"> <img src="images/quad_moments.PNG"></p>

The rotational movement of the quadcopter is obtained by finding the net moment about the x-axis, y-axis, and z-axis, individually. Movement of the quadcopter about the x-axis is called roll, movement about the y-axis is called pitch, and movement about the z-axis is called yaw.

The image above is a simplified version but the concepts carry over easily to 3-dimensions. In the image above, the left propeller exerts a force greater than the force exerted by the right propeller. The force exerted is perpendicular to the axis of rotation, causing the quad to roll. The same concept is true for pitch. 

Yaw is an interesting case, in that this motion is the result of the *reactive* rotational forces induced by the spinning propellers. Each propeller produces a rotational force that is **opposite** the direction of its spin. Yaw is directly propotional to the rotational acceleration of a propeller. 






Conversly, roll and pitch are caused