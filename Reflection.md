# Model Predictive Control Project Reflections
In this project I have learnt and implemented a Kinematic Model to predict path trajectory and actuations (steering angle and throttle) of a moving vehicle over a time horizon of 1 second. In below sections, I would discuss more about my implementation and reflect on project rubric points.

## Compilation

*Specification: Code must compile without errors with cmake and make.*

I have taken some liberty here to move functions `pi`, `deg2rad`, `rad2deg`, `polyeval` and `polyfit` defined in `main.cpp` files to new files `utils.h` and `utils.cpp` in order to reuse them in `MPC.cpp`.
Besides these functions, I have added a new version of `polyeval` that works with `Cpp::AD<double>` and is used inside Functor `FG_eval`. Added one more function `polyderive` to obtain a polynomial cofficients for derivative of given polynomial.

```c++
Eigen::VectorXd polyderive(Eigen::VectorXd coeffs) {
    Eigen::VectorXd d_coeffs(coeffs.size() - 1);
    for (int i = 0; i < d_coeffs.size(); i++) {
        d_coeffs(i) = coeffs(i+1) * (i+1);
    }
    return d_coeffs;
}
```

`utils.cpp` has been updated in `CMakeLists.txt` and code compiles well on my Ubuntu 16.04.3 LTS, i7 laptop.

```Makefile
set(sources src/utils.cpp src/MPC.cpp src/main.cpp)
```

## Implementation

### The Model

*Specification: Student describes their model in detail. This includes the state, actuators and update equations.*

In this project we have used a simple Kinematic bicyle model which predicts vehicle new state based on current state and actuators as input using following equations:

![](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20x_%7Bt%7D%20&plus;%20v_%7Bt%7D%20*%20cos%28%5Cpsi_%7Bt%7D%29%20*%20dt)

![](https://latex.codecogs.com/gif.latex?y_%7Bt&plus;1%7D%20%3D%20y_%7Bt%7D%20&plus;%20v_%7Bt%7D%20*%20sin%28%5Cpsi_%7Bt%7D%29%20*%20dt)

![](https://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_%7Bt%7D%20&plus;%20%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%20*%20%5Cdelta_%7Bt%7D%20*%20dt)

![](https://latex.codecogs.com/gif.latex?v_%7Bt&plus;1%7D%20%3D%20v_%7Bt%7D%20&plus;%20a_%7Bt%7D%20*%20dt)

Above Kinematics equation can estimate vehicle positions at time t+1 based on previous state at time t where t+1 - t = dt. These states also calculate vehicles heading direction. In this project, we are given waypoints to follow which we can use to find vehicle's ideal trajectory using a polynomial fit. However these equations alone are not sufficient for vehicle to stay on trajectory. For that, we need to determine its Cross Track Error (CTE) and error in heading direction. We need to minimize these errors in order to follow the trajectory of waypoints. CTE is measure of vehicle's distance from its trajectory given by following equation: 

![](https://latex.codecogs.com/gif.latex?cte_%7Bt%7D%20%3D%20y_%7Bt%7D%20-%20f%28x_%7Bt%7D%29)

At time t+1, CTE is calculated with following eqation:

![](https://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20cte_%7Bt%7D%20&plus;%20v_%7Bt%7D%20*%20sin%28e%5Cpsi_%7Bt%7D%29*%20dt)

Heading error is error in vehicle's heading direction w.r.t. optimal heading in waypoints trajectory.

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt%7D%20%3D%20%5Cpsi_%7Bt%7D%20-%20%5Cpsi%20des_%7Bt%7D)

It can be calculated as tangential angle of waypoints trajectory:

![](https://latex.codecogs.com/gif.latex?%5Cpsi%20des_%7Bt%7D%20%3D%20arctan%28f%5E%7B%27%7D%28x_%7Bt%7D%29%29)

Heading error is update in similar manner has heading itself with following equation:

![](https://latex.codecogs.com/gif.latex?cte_%7Bt%7D%20%3D%20y_%7Bt%7D%20-%20f%28x_%7Bt%7D%29)

We have the polynomial to follow, equations to update vehicle state and equations for errors. To follow the trajectory, we need to minimize the errors. But how does vehicle minimize these errors? By following actuations i.e. change in steering angle to minimize heading error and cross track error. But only steering is not sufficient as once minimal error is achieved, vehicle won't have any motive to move further and it will stay on that point forever. For Vehicle to keep moving, we also need to adjust throttle of the vehicle to keep it moving. It turns out to be an optimization problem which we solve using IPOPT libary to minimize our cost function. With MPC we predict vehicle states and its actuation over next N timesteps with smaller time interval dt. I have used following cost function in this project, with a multiplier factor associated with cost function to minimize different costs with different priorities. Below costs minimize CTE and heading error while trying to maintain a reference velocy. However maintaining velocity is at lower priority compared to CTE and heading error cost.

```c++
    // Reference State Cost
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.
    for (size_t t = 0; t < N; t++) {
      fg[0] += 1000 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1000 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
```

We don't want to frequently apply actuations, so we minimize them as well here. We also want to slow down while making sharp turns, so we add it to the cost. However this project is about going as fast as you can (safely), so I have subtracted minimum velocity to allow vehicle to maintain that velocity even on shapr turns.

```c++
    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
      // Penalize speed while making sharp turns
      fg[0] += 1000 * CppAD::pow(vars[delta_start + t] * (vars[v_start + t] - min_v), 2);
    }
```

Finally to make transitions smooth, I have used below cost functions minimize sudden change in steering angle and throttle. 

```c++
    // Smoothen out actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

### Timestep Length and Elapsed Duration (N & dt)

*Specification: Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

I started out with initial values for N=10 and dt=0.1 as used in [project Q&A](https://youtu.be/bOQuhpz3YfU) as I agreed upon horizon of 1 sec is good enough to make any sensible predictions. However with further turning of `dt`, I settled upon following values which given smoother transitions to the car:

```c++
size_t N = 20;
double dt = 0.05;
```

### Polynomial Fitting and MPC Preprocessing

*Specification: A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

I have preprocessed waypoints by transforming them in vehicle reference where vehicle is assumed to be at origin heading in x-axis i.e. `x=0, y=0, psi=0`. I have borrowed these translations from [project Q&A](https://youtu.be/bOQuhpz3YfU)

```c++
          // Transform the waypoints w.r.t. car's reference points
          Eigen::Map<Eigen::VectorXd> ptsx_e(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_e(&ptsy[0], ptsy.size());
          for (size_t i = 0; i < ptsx.size(); i++) {
            // In car's reference, car is at origin
            double shift_x = ptsx[i]-px;
            double shift_y = ptsy[i]-py;

            // rotate clockwise assuming that car's heading horizontally.
            ptsx_e[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
            ptsy_e[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
          }
```

Using these transformed waypoints, a polynomial fit was obtained which is further used to calculate Cross Track Error at origin (vehicle position) and error in headling direction:

```c++
          // Transform the waypoints w.r.t. car's reference points
          Eigen::Map<Eigen::VectorXd> ptsx_e(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_e(&ptsy[0], ptsy.size());
          for (size_t i = 0; i < ptsx.size(); i++) {
            // In car's reference, car is at origin
            double shift_x = ptsx[i]-px;
            double shift_y = ptsy[i]-py;

            // rotate clockwise assuming that car's heading horizontally.
            ptsx_e[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
            ptsy_e[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
          }
```

### Model Predictive Control with Latency

*Specification: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

To deal with 100 ms latency of actuation, I have used kinematic model equations as used by MPC to estimation state of the vehicle after 100 ms, which then serves as initial state for MPC.

```c++
  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];
  double cte0 = state[4];
  double epsi0 = state[5];
  double delta0 = state[6];
  double a0 = state[7];

  // Estimate initial state after actuation latency
  double x = x0 + v0 * cos(psi0) * latency;
  double y = y0 + v0 * sin(psi0) * latency;
  double v = v0 + a0 * latency;
  double cte = cte0 + v0 * sin(epsi0) * latency;
  double psi_delta = -v0 * delta0 * latency / Lf;
  double psi = psi0 + psi_delta;
  double epsi = epsi0 + psi_delta;
```

Note that some of these equations can't be computed without initial values for actuators, so I have used steering angle and throttle values provided by simulator and extened my state vector to include actuations like this:

```c++
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          ...
          Eigen::VectorXd state(8);
          state << 0, 0, 0, v, cte, epsi, steer_value, throttle_value;
          auto vars = mpc.Solve(state, coeffs);
```

## Simulation

*Specification: The vehicle must successfully drive a lap around the track.*

Below video is an output from my desktop simulation which captures 2 laps around the track. As you can see, throught the lap, vehicle stays only on drivable portion of the track. Top speed that car reaches is about **103 mph**!

[![project_video](http://img.youtube.com/vi/M0kiUkdXFkI/0.jpg)](http://www.youtube.com/watch?v=M0kiUkdXFkI)
