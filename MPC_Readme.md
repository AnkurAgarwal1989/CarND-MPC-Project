# Model Predictive Control

#### The Model

<u>The Model uses a 6 dimensional state consisting of:</u>

- vehicle's x and y position ($x,y$)

- orientation ($\psi$)

- speed ($v$)

- cross track error ($cte$)

- steering error ($e\psi$)

  ​						***All quantities are in vehicle's frame of reference.***

<u>There are 2 quantities that act as actuators.</u>

- throttle (accelerating and braking)
- steering values (-25 degrees to +25 degrees)

The update equations use a kinematic model with non-holonomic constraints.
$$
x_{t+1} = x_{t} + v_{t} * \cos(\psi_{t}) * dt \\
y_{t+1} = y_{t} + v_{t} * \sin(\psi_{t}) * dt \\
\psi_{t+1} = \psi_{t} - v_{t} / L_f * \delta_{t} * dt \\
v_{t+1} = v_{t} + a_{t} * dt \\
cte_{t+1} = f(x_{t}) - y_{t} + v_{t} * \sin(e\psi_{t}) * dt \\
e\psi_{t+1} = \psi_{t} - \psi_{des_t} - v_{t} * \delta_{t} / L_f * dt
$$

<u>A variety of costs have been considered for the optimization.</u> 

- One of note is the <u>smoothing of the state position in the y direction **(Line 91, MPC.cpp)**</u>. This makes the trajectory being followed smooth too.
- I expected the cost for smoothing delta to take care of this, but this seems to help the car travel fast enough while making smooth turns.

#### Timestep Length and Elapsed Duration (N & dt)

- Current value of <u>N and dt is 10 and 0.15</u> respectively. These values were chosen with some trial and error. We are essentially optimizing over N\*dt time. N larger than 15 makes the trajectory very long at higher speeds, which makes the optimizatio unstable. If dt is too long, we have coarser control. 
- I also tried values of N=15, 20 and 25. 20 and 25 did not seem to work very well. Also fewer N means smaller optimization space, which works.
- For dt, most values above 0.12 and under 0.18 seem to work.

#### Latency
- We need to account for 100 ms of latency. To do so, The initial state (global coordinates) was first propagated to future 100ms.
- These values are then used to transform waypoints from Map reference to Car reference.
- This is done in **Lines 134-138 in main.cpp**.


#### Polynomial Fitting and MPC Preprocessing

- An order 3 polynomial is being used. I tried an order 2 polynomial before, but that did not do very well near sharp turns. I did not spend too much time tuning that though.
- A pre-processing step is required to transform all x and y waypoints from map reference frame to car frame.

