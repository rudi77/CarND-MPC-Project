# CarND-Controls-MPC
<a href="url"><img src="https://github.com/rudi77/CarND-MPC-Project/blob/master/images/screenshotA.png" align="center" height="340" width="480" ></a>

This is the last project of the second term. The objective of this project is to implement a model predictive controller that is able to drive the car in the simulator safely around the lake track.

## Program Usage
- Clone the project from (https://github.com/rudi77/CarND-MPC-Project.git)
- Install all dependencies, see "Dependencies" below.
- Build the program, see "Basic Build Instructions" below
- Run it: ./mpc
- You can also run it with command line arguments:
```
rudi@pc1:/Udacity/CarND-MPC-Project/build$ ./mpc -h
Usage: mpc [-w weights | -v ref_v| -h]
CmdLine args description:
-w weights  The weights used in the cost function. A space separted string consisting of 7 weights (must be INTEGERS), e.g "1 10 15 1 1000 1 1"
-v ref_v    The reference velocity
-h          Help description
```
## Kinematic Model
In this project we implemented and used a kinematic model to model the vehicle. The kinematic model is a simplification of the dynamic model. It ignores forces, gravity and mass as opposed to a dynamic model and therefore maybe less accurate then the dynamic model.
The model consists of a state, actuators and state update equations:
### Vehicle's State
The vehicle's state can be described by:
- Position (x,y)
- Orientation (psi) 
- Velocity (v)
- Cross Track Error (cte): Offset from the center of the lane to the car.
- Orientation Error (epsi): Desired orientation subtracted from the current orientation.

All values but CTE and EPSI are provided by the simulator for every timestep t. CTE and EPSI are calculated.

I implemented a struct which encapsulates the vehicle's state. In a real system there will always be a certain delay when a command (actuation) is propagated through the system. Within this project we simulated this kind of delay. We assumed a delay of 100 ms. This delay is modeled as dynamic system and is embedded in the vehicle model. There are different approaches to handle this delay. One is to run a simulation starting with the current state of the vehicle model for the duration of the latency. During this time we assume that v is constant. This simulation is implemented in the `FutureState(latency)` method.
```c++
struct VehicleState
{
  double px;
  double py;
  double v;
  double psi;
  double cte;
  double epsi;
  double delta;
  double acceleration;

  // Returns the vehicle's state taking a certain delay into account.
  Eigen::VectorXd FutureState(double latency = 0.1)
  {
    Eigen::VectorXd state(6);

    px = 0 + v*latency;
    py = 0;
    psi = 0 -v*delta / Mpc::Lf*latency;
    v = v + acceleration*latency;
    cte = cte + v * sin(epsi) * latency;
    epsi = epsi + v*(-delta) / Mpc::Lf * latency;

    state << px, py, psi, v, cte, epsi;

    return state;
  }
};
```

The vehicle's position (px, py) and its orientation psi are initially always 0.

### Actuators
The actuators or the control inputs are the steering_angle and the throttle value. Both values are also provided by the simulator at each timestep t.

### Update Equations
<a href="url"><img src="https://github.com/rudi77/CarND-MPC-Project/blob/master/images/kinematic_model.png" align="center" height="340" width="480"></a>

These equations are implemented in the FG_eval class.


## Model Predictive Control
In Model Predictive Control (MPC) the task of following a trajectory can be seen as an optimization problem. The solution to this problem is an optimal trajectory. MPC involves simulating different actuator inputs, predicting trajectories and selecting the optimal trajectory, i.e. the one with the minimum cost. The optimal trajectory is re-calculated at every timestamp. Thus, it dynamically adapts its trajectory constantly.

### Polynomial Fitting and MPC Preprocessing
The simulator does not only provided the state information of the vehicle but also the reference trajectory in form of waypoints. These waypoints are used to fit a third order polynomial. The computed coefficients are then used to calculate the CTE and EPSI The following steps are carried out:
1.) Transform waypoint coordinates: The waypoint coordinates are given in the map's coordinate system which is different to the car's one. The coordinate transformation is done in the  `ransform_coordinates(...)` function in the main.c file:
```c++
vector<double> transform_coordinates(double x, double y, double psi, double x0, double y0)
{
  auto cos_psi = cos(psi);
  auto sin_psi = sin(psi);

  // Rotation + Translation
  auto x_new = cos_psi * (x0-x) + sin_psi * (y0-y);
  auto y_new = -sin_psi * (x0-x) + cos_psi * (y0-y);

  return { x_new, y_new };
}
```

2.) Compute the third order polynom.
3.) Calculate the CTE and EPSI: This is directly done in the main function:
```c++
  ...
  const auto cte  = polyeval(coeffs, 0);
  const auto epsi = -atan(coeffs[1]);
  ...
```

### MPC Algorithm
1.) Define duration of trajectory T: This means that we have to define N and dt where N is the number of steps and dt is the duration of one timestep. A large N and a small dt provides very accurate results but also increases the computational cost and latency. A smaller N and a larger dt returns solutions which are more inaccurate but increases responsiveness of the controller. After some trial and error I defined N=15 and dt=0.05.

2.) Define the vehicle model and constraints.

3.) Define the cost function: The cost function should minimize the CTE and EPSI and is defined as follows.

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

