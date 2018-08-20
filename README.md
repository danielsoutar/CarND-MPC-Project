# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Hi there, and welcome to my final project for Term 2 of the Udacity Self-Driving Car Engineer Nanodegree.

I'm apparently required to write up some parts of this project to show understanding of what I've done. What a crazy idea. 

## Student describes their model in detail. This includes the state, actuators and update equations.

Model Predictive Control (MPC) is a more advanced control algorithm than PID control. The reason is because we are now modelling the process and simulating this process over a given time horizon, enabling MPC to predict the future, which a PID controller cannot do.

MPC requires several components. We read in the current state, acquired by our sensors (e.g. lidar, radar, stereo camera, odometry data). We define the model that models the process (in our case, how a vehicle's trajectory evolves over time using the bicycle model). We define the constraints on our actuators (basically, what our vehicle is physically capable of since we can't immediately turn 270&deg). We finally define a cost function that is to be minimised. It is this cost function that we tune to get better performance and specify desired behaviour, since the cost function may reward or punish certain behaviours, like rewarding harder acceleration out of corners, or punishing abrupt lane changes.

The solver that is used by MPC takes in this initial state, the model, constraints, and cost function to produce a vector of (constrained) actuations that minimises our cost function. Note that this is often a _locally_ optimal solution! We then execute the first actuations, and discard the rest of the vector. Although wasteful, this is important and necessary since a) our model is only approximate, b) there is a degree of motion noise that our solver doesn't necessarily take into account and c) the external environment may dynamically influence our trajectory (for instance, a car overtaking us likely implies we should maintain or decrease our speed). 

The actual model is defined by specifying how a variable (such as our position in x-y coordinates) is constrained by the previous timestep. This has a Markovian element, since we are only basing our prediction of the variable at a given timestep based on the timestep immediately prior, independently of other timesteps. This simplifies the logic of the model, as well as reducing memory requirements (both good things). 

## Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

_N_ and _dt_ are important because they determine the time horizon over which we simulate the trajectory and minimise the cost function. If _N_ is large and _dt_ is small, we have a longer time horizon to take account of and our motion is more granular, as we take many steps due to _dt_. This is better for accuracy, but more computationally intensive and potentially longer to get a good result for. The converse is true if _N_ is small and _dt_ is large.

My reasoning was that we don't need to predict much beyond around a second. This has the added benefit of shortening the trajecory, so computation-wise it is not too bad. A bigger reason for this was because a longer trajectory also increases the likelihood of getting some seriously weird outputs - even the Udacity example (the GIF shown to us to demonstrate the final result) has a couple of timesteps where the planned trajectory goes off with a mind of its own. This reflects the "local" in the "locally optimal" solution that our solver (provided by the IPOPT package) provides. By shortening the time horizon to evaluate, our trajectory becomes increasingly linear and so we have less variability in the result. I decided that _dt_ should just be the same as the latency we have to deal with, being 0.1 seconds, or 100 milliseconds. This seemed to work well, although whether _dt_ for the solver is better the closer it is to our actual latency is something I'm unaware of.

At any rate, these two parameters worked out quite well, and it was mainly a question of tuning the cost function from there onwards.

## A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints are converted to the vehicle's own coordinate system, since they are initially in the global coordinate system. This meant that I couldn't even spot the damn waypoints when I first tried rendering them! After going on the internet to check this, the penny dropped and I converted them using the homogenous transform. This simplified the rest of the code, which was handy. 

## The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The latency (which is set as a parameter) is also used to update the initial state before passing it to the solver. The reason why I do this is because by the time the solver actually solves for the optimal trajectory and returns the requisite actuations, the lag for these actuations to propogate through the system means they are slightly behind. So I simply adjust the initial state to account for the latency, so that when the actuations are executed they are done so at greater accuracy to the actual state.

As the below video shows, the vehicle shifts along in a far smoother fashion to its PID-controlled sibling, albeit I think(?) at a slightly slower pace. It drives much more consistently, which is good. No doubt the increase in the complexity of the model allows it to have more fine-tuned control over the vehicle's behaviour in different situations. 

[Final controller no simplifying](./final_controller_no_simplifying.mov "Final controller, no simplifying")

## And...

Yes, I tried optimising a little further. Although the MPC controller delivers a better all-round performance, you would damn hope so considering it has to muck around with some considerably more intensive solving. The PID controller is laughably simple in comparison and over the course of a drive I suspect would have massively lower power consumption. In certain scenarios, particularly when travelling in a fairly steady-state fashion along a highway, one might reasonably ask whether this additional complexity and number-crunching is really necessary. 

For these reasons, I decided to try simplifying the controller, particularly along the straight(er) bits. The track is probably not the best place to do this since only the bridge is truly straight, and is immediately followed by a fairly tight corner. I initially tried to reduce the order of the polynomial to pass to the solver (which by default is a third-order polynomial) if the average steering angle was sufficiently close to 0. This worked reasonably well, as the below video shows, although there are moments where the reference and planned trajectories are 'bumped' more so than if the trajectories were constantly cubic.

[Linearising the waypoints](./linearising_waypoints.mov "Linearising the waypoints")

I also decided to try increasing the latency if the reference trajectory was sufficiently linear (I check the absolute-valued sum of the quadratic and cubic terms) over the last _t=12_ timesteps, since I assume (reasonably, as it turns out) that the vehicle stays fairly close to the reference trajectory already anyway. Additionally, if the reference trajectory is fairly straight, then it's reasonable to assume that it will generally remain as such for a while. This latency would also feed into the update of the initial state by modifying _dt_, so there wouldn't be any disconnect between the solver and the initial state equations. My reasoning for this approach was that the most efficient code, by definition, does not run at all. So if we simply cause the current thread to sleep, no computation is done, and that's as good as it gets!

This hare-brained scheme actually seemed to work remarkably well, although there are one or two 'moments' in the last few corners where the car gets a bit jittery, whereas the standard implementation is smoother. As the below video shows, however, the general result is excellent - apart from the apices of the corners, the car remains in this 'cooldown' mode and performs near-identically in most cases, even though we have 50% greater latency between actuations. Perhaps on longer straights the effect could be extended even further. Maybe a truly integrated solution would detect a corner up ahead, perhaps via camera, alerting the controller to exit 'cooldown' mode to smoothen out turning. In any case, it works just fine here. Great stuff indeed!

[Warm-up and cool-down](./warm_up_cool_down.mov "Warm-up and cool-down")

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

