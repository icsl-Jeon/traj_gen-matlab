## traj_gen :  a continuous trajectory generation with simple API

*traj_gen* is a continuous trajectory generation package where <u>high order derivatives</u> 
along the trajectory are minimized while satisfying waypoints (equality) and axis-parallel box constraint (inequality). The objective and constraints are formulated in *quadratic programming* (QP) to be updated in real-time. 

- To parameterize a trajectory, we use two types of curve: 1) **piecewise-polynomials** [1,2] and 2) **a sequence of points** [3]. 
The difference is optimization variables.   
   
- In this package, we use a term **pin** to refer the constraints. *Fix-pin* refers a waypoint constraint, 
and *loose-pin* denotes a axis-parallel box constraint. The pin is a triplets (time (t), order of derivatives (d), value (x)) where x is 
a vector in case of fix-pin while two vectors [xl xu] for the loose-pin.  

We implemented traj_gen in Matlab and C++(upcoming ~ the end of Mar).
Also, we plan to provide ROS support such as the [previous version](https://github.com/icsl-Jeon/traj_gen)
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/tutorial.gif">
</p>



  ### Reference 

[1] Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors." *2011 IEEE International Conference on Robotics and Automation*. IEEE, 2011.

[2] Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." *Robotics Research*. Springer, Cham, 2016. 649-666.

[3] Ratliff, Nathan, et al. "CHOMP: Gradient optimization techniques for efficient motion planning." *2009 IEEE International Conference on Robotics and Automation*. IEEE, 2009.