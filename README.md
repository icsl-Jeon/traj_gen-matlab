## traj_gen :  a continuous trajectory generation with simple API (MATLAB)

<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/tutorial.gif">
</p>

*traj_gen* is a continuous trajectory generation package where <u>high order derivatives</u> 
along the trajectory are minimized while satisfying waypoints (equality) and axis-parallel box constraint (inequality). The objective and constraints are formulated in *quadratic programming* (QP) to cater the real-time performance. C++ implementation can be found [here](https://github.com/icsl-Jeon/traj_gen). 

- To parameterize a trajectory, we use two types of curve: 1) **piecewise-polynomials** [1,2] and 2) **a sequence of points** [3]. 
The difference is optimization variables.   

  a. **Piecewise-polynomials (polyTrajGen class)** : It defines the primitive of the curve as polynomical spline. The optimization target is either *polynomial coefficients* [1] or *free end-derivatives* of spline segments [2] (can be set in constructor). In general, the latter has fewer optimization variables as it reduces the number of variable as much as the number of equality constraints.    
  b. **A sequence of points (optimTrajGen class)** : It does not limit the primitive of the curve. The optimization target is a finite set of points. The final curve is defined as a linear interpolant of the set of points. The point density (# of points per time) should be set in the constructor. Instead of unlimited representation capability of a curve, the size of optimization is driectly affected by the point density.          
  
- In this package, we use **pin** to accommodate the two constraints: equality (*fix pin*) and inequality (*loose pin*). Pin can be imposed regardless of the order of derivatives. *Fix-pin* refers a waypoint constraint, 
and *loose-pin* denotes a axis-parallel box constraint. The pin is a triplets (time (t), order of derivatives (d), value (x)) where x is 
a vector in case of fix-pin while two vectors [xl xu] for the loose-pin.  

 - We implemented traj_gen in Matlab and plan also [C++(upcoming ~ the end of Mar)](https://github.com/icsl-Jeon/traj_gen). In case of 2D trajectory generation in Matlab, we provide interactive pin selection (see poly_example/main2D).


### Matlab API quick start (poly_exmaple/main3D.m)
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/quick_start.png">
</p>

Also, check multiple examples in ./poly_example and ./optimal_example. The below is an example **main2D.m** where we can interactively select pin information. 

<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/2d_tutorial.gif">
</p>




## Detailed description on API

### 1. Parameters (arguments in constructor and setDerivativeObj method)
- **Common** 
  - *Knots (t1,...,tM)* : time knots. In case of polyTrajGen, it defines the segment intervals in time domain. *The fix-pin can be imposed on these time knots* (no limitation for loose pin). In case of optimTrajGen, the time knot is just the start time and end time as it is not defined on a set of time sgements. 
  - *Penality weights for the integral of derivative squared* : as the objective of our optimization is the weighted sum of the integral of derivative squared, we have to define the importance weight for each component. This value ws = [w1 w2 ... wd] can be set as the argument of setDerivativeObj(ws). For example, if you want to implement a minimum snap trajectory generation, then set ws = [0 0 0 1] while ws = [0 0 1] for minimum jerk trajectory generation.
  
- **polyTrajGen**
  - *Polynomial order (N)* : the order of all the polynomial segments. Although it can increase the power of representation of a curve, the size of optimization variables increases in proportion to (N x M).   
  
  - *Optimization target* (*'end-derivative'* or *'poly-coeff'*) : the target of optimization. 'poly-coeff' method sets the coefficients of polynomials as optimization variables in a similar way with [1]. The 'end-derivative' sets the optimization variables as the free derivative values of each end point on a polynomial segment. As this method is equivalent to plugging the equality constraints (fix pin and continuity) to optimization problem, it reduces the optimization dimension at the cost of inversion of a mapping matrix. The dof of a segment is thus (poly_order - # of fix pins on the segment - maximal continuity). For the details, please refer [2].   
    
  - *Maximum continuity* : the maximally imposed continuity order between neighboring segements. Higher value of this parameter enhances the quality of smoothness. However, too high value of this value restricts the dof for optimization, downgrading the opitimization result.     
  
  
- **optimTrajGen**
   - *Point density* : the number of posed points per time [s]. For long-ranged trajectory, thus, the total number of variables will increase leading to the burden of the optimization. 



  ### Reference 

[1] Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors." *2011 IEEE International Conference on Robotics and Automation*. IEEE, 2011.

[2] Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." *Robotics Research*. Springer, Cham, 2016. 649-666.

[3] Ratliff, Nathan, et al. "CHOMP: Gradient optimization techniques for efficient motion planning." *2009 IEEE International Conference on Robotics and Automation*. IEEE, 2009.
