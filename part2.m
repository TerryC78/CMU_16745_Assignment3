% Part 2
clc; clear all; close all;

% Initialize limp
limp.plan = Parameters('plan001');  % read footstep plan
% Suppose the COM locations are given
nSteps = length(limp.plan.p_x); % the number of steps
p_x = limp.plan.p_x';
p_y = limp.plan.p_y';
% Initialize the durations
duration = limp.plan.time';

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
% TODO: Sideway oscillations are bounded by p_y


% (3) Keep step size reasonable: constraint on p_x
footstep_size = zeros(1, nSteps-1);
for i = 2:nSteps
    footstep_size(i-1) = p_x(i) - p_x(i-1);
end
% Set constraint: footstep_size <= 1 (parameter) >= 0
% Minimize the variance also
footstep_var = var(footstep_size);

% (4) Penalty on swings on both feet
num_swings = floor(nSteps / 3);
swing_penalty_right = zeros(1, num_swings);
right_counter = 1;
swing_penalty_left = zeros(1, num_swings);
left_counter = 1;
for i = 2:(nSteps-1)
    if mod(i, 2) == 0 && (i+2) <= (nSteps-1)
        swing_distance_right = p_x(i+2) - p_x(i);
        swing_duration = sum(duration(i:i+2));
        swing_penalty_right(right_counter) = (swing_distance_right/swing_duration^2)^2;
        right_counter = right_counter + 1;
    end
    
    if mod(i, 2) ~= 0 && (i+2) <= (nSteps-1)
        swing_distance_left = p_x(i+2) - p_x(i);
        swing_duration = sum(duration(i:i+2));
        swing_penalty_left(left_counter) = (swing_distance_left/swing_duration^2)^2;
        left_counter = left_counter + 1;
    end
end

total_swing_penalty = sum(swing_penalty_right) + sum(swing_penalty_left);

total_cost = constant_vel - 0.5 * sideway_oscillation + footstep_var + total_swing_penalty;
