%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   Assignment 3 -- Dynamic Optimization
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear all; close all;

G  = 9.81; % [m/s^2]
m  = 1;    % [kg] 
z  = 1;    % [m]
x  = 0;    % [m]
y  = 0;    % [m]
xd = 0;    % [m/s]
yd = 0;    % [m/s]
xdd = 0;   % [m/s^2]
ydd = 0;   % [m/s^2]

% initialize limp
limp.plan = Parameters('plan001');  % read footstep plan
limp.x = [];
limp.y = [];
limp.u_x = [];
limp.u_y = [];
limp.footx = [];
limp.footy = [];

[c1, c2] = size(limp.plan.time); 

for i = 1:c1

    sim('LIMP.slx');
    [x, y, xd, yd, u_x, u_y, limp] = processData(Sx, Sy, Sxd, Syd, Su_x, Su_y, i, limp);
    
end

figure(1)
plot(limp.footx,'b','LineWidth',2)
hold on
plot(limp.x,'r','LineWidth',2)
hold on
plot(limp.u_x,'cyan','LineWidth',2)
grid on
legend('foot location','COMx','COPx','Location', 'southeast')
title('COM trajectory in X')
xlabel('Simulation Time ')
ylabel('Trajectory [m]')

figure(2)
plot(limp.footy,'b','LineWidth',2)
hold on
plot(limp.y,'r','LineWidth',2)
hold on
plot(limp.u_y,'cyan','LineWidth',2)
grid on
legend('foot location','COMy','COPy','Location', 'northwest')
title('COM trajectory in Y')
xlabel('Simulation Time ')
ylabel('Trajectory [m]')