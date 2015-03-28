# Dynamic-Optimization-Assigment3
CMU RI 16-745: Dynamic Optimization: Assignment 3

This assignment explores trajectory optimization. Given a set of footstep locations and timing, you find an appropriate center of mass trajectory. We use a simplified model, the linear inverted pendulum model (LIPM).

The LIPM model

Google "lipm linear inverted pendulum model" for more information, and solution methods for this assignment. For example, this paper describes how to do the assignment using DDP.

In this assignment the robot consists of a point mass and massless legs. We will maintain the body at constant height z. We assume no double support except at the beginning and end of the trajectory. The LIPM dynamics in the x direction (forwards) are given by:

m * xdd = Fx
m * xdd = Fz*(x - p_x)/z
m * xdd = m*G*(x - p_x)/z 
so
xdd = (x - p_x) * G/z
m is the mass, x is the COM location in the x direction, xdd is the COM acceleration, Fx is the horizontal force from the leg, Fz is the vertical force from the leg, p_x is the foot location in the x direction and G is the gravitational constant. We will add ankle torque actuation, but use the offset of the center of pressure (COP) that it causes as the actuation u_x:
xdd = (x - p_x + u_x) * G/z
There is a corresponding equation for the dynamics in the y direction (sideways):
ydd = (y - p_y + u_y) * G/z
The cost function to optimize is
(x - p_x)^2 + xd^2 + 30*u_x^2 + (y - p_y)^2 + yd^2 + 30*u_y^2
xd is the COM velocity in the x direction.
Here is a footstep plan, which sets footstep timing, p_x, and p_y. The columns are duration (sec), p_x, p_y, and stance_type (2 = double support, 0 = right stance, 1 = left stance).

1.5 0 0 2
0.5 0 -0.15 0
0.5 0.7 0.15 1
0.5 1.4 -0.15 0
0.5 2.1 0.15 1
0.5 2.8 -0.15 0
0.5 3.5 0.15 1
1.5 3.5 0 2
Here are two plan files in this format: plan001 and plan002.
Here is a optimized trajectory for the above footstep plan.















What to do?

Part 1) Develop a computer program to find an optimal COM trajectory (x(t) and y(t)) and COP offset trajectory (u_x(t) and u_y(t)), given a footstep plan.

Part 2) The footstep timing is fixed in the current footstep plan. Develop a computer program to find an optimal COM trajectory and COP offsets, while also optimizing footstep timing.

The first thing to figure out is what is a reasonable optimization criterion:

Trying to keep forward or total velocity constant?
Trying to keep the side to side oscillation going?
Trying to keep the step size reasonable?
Assuming a penalty on swinging the foot of
(distance/(time^2))^2
where distance is the swing distance, and time is the time taken to swing the foot?
Part 3) Develop a computer program to find an optimal COM trajectory and COP offsets, while also optimizes footstep location and timing. Assume quadratic cost functions for each footstep location (p_x_i and p_y_i) are given ((p_x_1 - p_x_1_d)^2, etc.).

Part 4) Thought Question: The number of footsteps and footstep location is fixed by the footstep plan. How would you set up an optimization problem and algorithm to optimize the number of footsteps, footstep timing, and footstep location. How would terrain information be presented to the planner?

What to turn in?

You can work in groups or alone. Generate a web page describing what you did (one per group). Include links to your source and any compiled code in either .zip, .tar, or .tar.gz format. Be sure to list the names of all the members of your group. Mail the URL of your web page to cga@cmu.xxx and snagaval@andrew.xxx. [You complete the address, we are trying to avoid spam.] The writeup is more important than the code. What did you do? Why did it work? What didn't work and why?
The TA will have office hours after class Monday and Wednesday.

Questions


