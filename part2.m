% Part 2
clc; clear all; close all;

% Initialize limp
limp.plan = Parameters('plan001');  % read footstep plan
% Suppose the COM locations are given
nSteps = length(limp.plan.p_x); % the number of steps
p_x = limp.plan.p_x';
p_y = limp.plan.p_y';
% Create a vector of durations
duration = zeros(nSteps, 1)';
% Initialize the durations
for i = 1:nSteps
    duration(i) = limp.plan.time(i);
end

% (1) Make the velocity at each footstep (x-direction) more constant (less variable)
x_d = zeros(nSteps-1, 1)'; % velocity at each footstep
for i = 1:(nSteps-1)
    x_d(i) = (p_x(i+1) - p_x(i)) / duration(i);
end
mean_x_d = mean(x_d); % the average velocity
% Define the constant velocity criteria
constant_vel = sum((x_d - mean_x_d).^2); % do I need to consider 0 velocities at start and end?

% (2) Keep the sideway oscillations going
% TODO: How to relate oscillations to timings?
