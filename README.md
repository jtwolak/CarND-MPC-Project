



# Self-Driving Car Engineer Nanodegree

**Term2: CarND-Controls-MPC - Project 5** 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Description

The goal of this project is to implement a Model Predictive Control to drive a car around the track in the simulator. 

A video showing successful run around the track is provided in the following file:  mcp_result.mp4

## The Model

The model used in this project is a  Kinematic Model that ignored effects of dynamic forces such as tire forces gravity and mass.  The model is described by a set of equations that capture the state of the vehicle that takes into account the position of the vehicle that is tracked as x and y, orientation of the vehicle and its velocity. The Kinematic Model describes how the vehicle state changes based on actuators such steering wheel and throttle and break pedal that is modeled as a single actuator with negative values signifying breaking and positive values signifying acceleration. So for actuators the model uses delta to show the steering angle and "a" for acceleration both positive and negative (breaking). The Kinematic Model is described by the following equations:

State: [x, y, psi, v]

- x,y 	- position of the car, 
  - psi- heading direction
- v      - velocity

Control Inputs: [delta, a]

- delta - 
- a - acceleration

    /*
    	 * Model Equations:
    	 *
    	 * x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
    	 * y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
    	 * psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
    	 * v_[t] = v[t-1] + a[t-1] * dt
    	 * cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
    	 * epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
    	 */
- cte - cross-track error
- epsi- orientation error
  - Lf- distance between center of mass of the vehicle and the front wheels

## Timestep Length and Elapsed Duration (N & dt)

The time T=N*dt defines the prediction horizon. The values chosen for N and dt are 10 and 0.05 respectively.  The prediction horizon is an important parameter, especially when we take into account the desired vehicle speed and road trajectory.  After some experimentation we settled on 10/0.05 because these values  provide the desired result for the  selected constant speed of 50mph.   Larger values of N such as 12, 15 and 20 were also tried but in some cases the final effect was not as good as for N=10 and larger N values require more computation power. The dt value of 0.05 was influenced by the Mind The Line exercise and proved to be convenient for calculation of offset used to compensate for the 100ms latency.

## Polynomial Fitting and MPC Processing

Before polynomial fitting we calculate the waypoints by transforming the trajectory points received from the simulator from map to car coordinate system using the  following equations:


	Ptsx(i) = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
	Ptsy(i) = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);

- ptsx, ptsy	- trajectory points in the map coordinate system
- Ptsx, Ptsy       - transformed waypoints in the car coordinate sytem
- px,py - position of the car
- psi - heading direction

Note: In the car coordinate system the vehicle coordinates are at the origin (0,0) and the orientation angle is also 0.

Then a third order polynomial is filled to the waypoint to obtain the polynomial coefficients:

		  auto coeffs = polyfit(Ptsx, Ptsy, 3);
Subsequently the cross-check and orientation errors are calculated

		 double cte = polyeval(coeffs, 0);
	      double epsi = -atan(coeffs[1]);   
Note: The orientation error epsi is -atan(c1 + c2*x + c3* x^2), but since the car is always at x=0 we can simplify the equation to just -atan(c1).

The projected trajectory and actuator values are calculated using MPC . Since the calculations are done in car coordinate system and the car is always at 0.0 coordinates with orientation equal to zero we can simplify the state to 

      state << 0, 0, 0, v, cte, epsi; 

Finally MPC.Solve() routine is called to obtain the projected trajectory along with the steering angle and throttle values:

	solution = mpc.Solve(state, coeffs); 
The predicted X and Y position values are placed in the output vector returned by the Solve() at the MPC_X_OFST and MPC_YOFST positions respectively.

	mpc_x_vals.push_back(solution.at(out_index + MPC_X_OFST) );
	mpc_y_vals.push_back(solution.at(out_index + MPC_Y_OFST));
The predicted throttle and steering angle values can be found at MPC_THROTTLE_OFFSET and MPC_SPEED_OFST offsets respectively for each of the predicted points.

	throttle_vals.push_back(solution.at(out_index + MPC_THROTTLE_OFFSET) );
	steer_vals.push_back(solution.at(out_index + MPC_SPEED_OFST) );

## Model Predictive Control with Latency

To deal with the latency a steering angle and throttle values calculated for the time interval in the future equal to the latency are provided to the simulator instead of the current values.

We also constraint the first actuator value to its previous value like code below:

```
// set constraints in MPC:Solve()
    for (int i = delta_start; i < delta_start + fixed_steps; i++)
    {
        vars_lowerbound[i] = prev_delta;
        vars_upperbound[i] = prev_delta;
    }

    for (int i = a_start; i < a_start + fixed_steps; i++)
    {
        vars_lowerbound[i] = prev_a;
        vars_upperbound[i] = prev_a;
    }
```