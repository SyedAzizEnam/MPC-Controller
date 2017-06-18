# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

![alt text](http://url/to/img.png)

# MPC
The model predictive controller (MPC) is a common controller that tends to perform better than other controllers such as PID controllers. This is because MPC takes into account how the vehicles trajectory will change based on certain actuations. MPC optimizes a cost over a trajectory rather than a simple state like traditional PID controllers. 

# The Model
The model of the predictive controller is a basic kinematic model that assumes constant velocity and yaw except when changed by the actuators. 

The simulator provides us the state of the vehicle long with a some points along the middle fo the lane. We can fit a polynomial to these opints to genereate an optimal trajectory which we can use to calcualte cte(cross track error) and espi (error psi). Given a state < x, y, psi, cte, epsi> at time t, we can predict what the state will be at t+1 with the following equations:

$$
x_(t+1) = x_t + v_t*cos(\psi_t)*dt
y_(t+1) = y_t + v_t*sin(\psi_t)*dt
\psi_(t+1) = \psi_t + \frac{v_t}{L_f}*\delta_t)*dt
v_(t+1) = v_t + a_t*dt
cte_(t+1) = f(x_t) - y_t + (v_t*sin(e\psi_t)*dt
e\psi_t+1 = \psi_t - \psi_dest + (\frac{v_t}{L_f}*\delta_t)*dt)
$$

The model is describe in the following constraints in MPC.cpp
```
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
              cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
              epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
```
# Polynomial Fitting and Preprocessing

In order to simplfy some calculations we shift and rotate the points given to us by the simulator to the cars reference coordinates.

```
for (int i = 0; i< ptsx.size(); i++)
{
  //translate
  double t_x = ptsx[i] - px;
  double t_y = ptsy[i] - py;

  //rotate
  ptsx[i] = t_x * cos(-psi) - t_y * sin(-psi);
  ptsy[i] = t_x * sin(-psi) + t_y * cos(-psi);
}
```

# The Cost Optimization
Once we have trajectories we can associate costs with them. We obviously want to minimize the cte and epsi but we also can incorporate other costs for some desired behavior. For example we can include the difference in speed from a desired speed, prefer trajectories with less actuations, and prefer smooth transistions between actuations. 

The cost is describe in the following equations in MPC.cpp
```
    fg[0] = 0;
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

# Timestep Length and Elapsed Duration (N & dt)
When we generate trajectories we do so by calulating the predicted state of the vehicle at certain time steps. We have two parametets we can tune here the number of steps N and the tiem gaps between the steps dt. I chose to use N = 9 and dt= 0.1. Other values for N where tried but higher values of N led to oscillations and lower values fo N led to leaning towards one side.  

# Model Predictive Control with Latency

The simulator also provides us a way to simluate time latency experienced in real world systems. The latency is set to about 100ms. To handle latency the actuators were constrained to the previous actuations for one time step for the trajectory simulations. 

We do this by setting the upper and lower bounds of the actuators to the previous actuations in the following lines of code: 
```
  for (int i = delta_start; i < delta_start + latency_steps; i++)
  {
    vars_lowerbound[i] = previous_delta;
    vars_upperbound[i] = previous_delta;
  }
  
  ...
  
  for (int i = a_start; i < a_start + latency_steps; i++)
  {
    vars_lowerbound[i] = previous_a;
    vars_upperbound[i] = previous_a;
  }
```
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
