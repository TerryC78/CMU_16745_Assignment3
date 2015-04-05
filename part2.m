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
midpoint = zeros(1, nSteps-1);
for i = 1:(nSteps-1)
    midpoint(i) = 0.5 * (p_y(i) + p_y(i+1));
end
sideway_oscillation = sum((p_y(1:(nSteps-1)) - midpoint).^2);

% (3) Keep step size reasonable: constraint on p_x
footstep_size = zeros(1, nSteps-1);
for i = 2:nSteps
    footstep_size(i-1) = p_x(i) - p_x(i-1);
end
% Set constraint: footstep_size <= 1 (parameter) >= 0
% Minimize the variance also
footstep_var = var(footstep_size);

% (4) Penalty on swing
num_swings = floor(nSteps / 3);
swing_distance_right = zeros(1, num_swings);
swing_distance_left = zeros(1, num_swings);
for i = 2:(nSteps-1)
    
end




