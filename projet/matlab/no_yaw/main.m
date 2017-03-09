%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Filename:   main.m
%   Author:     Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%   Class:      MTH8408
%   Description:Implementation of "Minimum Snap Trajectory Generation
%               and Control for Quadrotors" but simplified
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

k_r = 4;    % Order of derivative of the position
k_psi = 2;  % Order of derivative of the yaw
mu_r = 1;   % Non-dimentionalization constant for the position integral
mu_psi = 1; % Non-dimentionalization constant for the yaw integral
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
            % Can also be seen as the number of trajectory pieces
states = 3; % x y z, let's ignore the yaw for now...

t = [0 1 2 3];

% Waypoint constraints
% X axis
w(:, :, 1) = [  0   1   1   0; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints

% Y axis
w(:, :, 2) = [  0   0   1   1; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints

% Z axis
w(:, :, 3) = [  1.5 1.5 1.5 1.5; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints
tic
H = buildh(n, m, mu_r, mu_psi, k_r, k_psi, t);
[Aeq, beq] = buildEqualityConstraints(n, m, k_r, k_psi, t, w);
options = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex', 'Display', 'iter');
coeffs = quadprog(H, [], [], [], Aeq, beq, [], [], [], options);
toc

