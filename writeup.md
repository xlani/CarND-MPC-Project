# MPC Project #

This writeup addresses the points of the project rubrics for the MPC (Model Predictive Control) Project.

## The Model

The model state vector is a 6x1 vector with the following components

```
state[0] = x
state[1] = y
state[2] = psi
state[3] = v
state[4] = cte
state[5] = epsi
```

where cte is the __cross track error__ `y - f(x)` and __epsi__ is the error `psi - psi_desired`.

The model has two actuators for steering and acceleration, with the corresponding variables `delta` and `a`.

The state is updated as following:

```
x1 = x0 + v0 * CppAD::cos(psi0) * dt;
y1 = y0 + v0 * CppAD::sin(psi0) * dt;
psi1 = psi0 + v0 * delta0 / Lf * dt;
v1 = v0 + a0 * dt;
cte1 = (f0 - y0) + (v0 * CppAD::sin(epsi0) * dt);
epsi1 = (psi0 - psides0) + v0 * delta0 / Lf * dt;
```

where `x0` stands for `x` at time `t` and `x1` stands for `x` at time `t=t+dt`. `Lf` is the length from front to center of gravity of the car.


## Timestep Length and Elapsed Duration (N & dt)

I started out with timesteps `dt = 0.1` and `N = 20`. That produced quite well results for no latency and lower speeds. After turning on latency I tried to de- and increase `dt` and `N` with not really improving the observed oscillations of the car. Then I switched my approach for handling latency (see chapter Model Predictive Control with Latency below).

Because of taking the mean over 3 timesteps of the solutions found by Ipopt for both actuator commands, I wanted to make sure to have `dt` conform with latency and kept it at `dt = 0.1`. This covers the time of 100 - 300 milliseconds and takes care for latency of 100ms very well.

It turned out that it was possible to turn down N a little bit. So finally I ended up with `dt = 0.1` and `N = 10` for a desired target velocity of 60 mph.

## Polynomial Fitting and MPC Preprocessing

For preprocessing I decided to transform the given waypoints from the map coordinate system to the vehicle coordinate system. This simplifies the __cross track error__ (cte) to `cte = -coeffs[0]` and the error __epsi__ to `epsi = -atan(coeffs[1])`, after fitting the transformed waypoints to a polynomial via function `polyfit` with the resulting coefficients `coeffs`.

Additionally this has the advantage, that three of the six input values for the simulator are set to `0`:

```
//set state in car coordinates as input for solver
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;
```

## Model Predictive Control with Latency

To deal with latency I tried first using the approach proposed in the Udacity lessons. I "simulated" the state 100ms in the future for constant steering inputs and used this new state as input for the optimizer. Unfortunately this caused me lot of problems with oscillating behaviour of the car (maybe because I could not find the right values for timestep `dt`).

While searching for ways to handle this problem I found via SDC-slack an approach provided by user wsteiner ([source](https://github.com/WolfgangSteiner/CarND-MPC-Project)) and implemented this. The approach does use the original state at `t=0` for input and takes the mean of the solution of the first three timesteps of the actuators steerings and gives that back as steering commands for `t=0`. It turned out that this worked really well off the start.

The last steps were fine tuning of the cost function. The cost function consists out of three major parts:

- Reference states
- Use of actuators
- Gap between sequential actuations

I found out, that it was enough to only penalize the use of both actuators, with focus strongly on steering. I chose the weighting factors `1000` for steering and `2` for acceleration (this prevented the car to toggle all the time between accelerating and braking) and achieved a smooth behaviour of the car for my desired target velocity `v_ref = 60 mph`.
