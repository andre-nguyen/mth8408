%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Filename:   main.m
%   Author:     Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%   Class:      MTH8408
%   Description:Implementation of Minimum snap trajectory generation but
%   modified to follow the formulation by Adam Bry/Charles Richter. The
%   only difference is that the flat outputs are each treated as individual
%   optimization problems, so we have multiple problems to solve instead of
%   one big one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;

k_r = 4;    % Order of derivative of the position
k_psi = 2;  % Order of derivative of the yaw
mu_r = 1;   % Non-dimentionalization constant for the position integral
mu_psi = 1; % Non-dimentionalization constant for the yaw integral
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
states = 3;

% For a quadratic optimization problem of the form
%   min c'Hc + f'c
% subject to
%       A * c <= b
%       Aeq * c = beq
% We know the linear term f' is nul and c is a 4nm x 1 vector containing the
% coefficients of the polynomials. So now we have to build the matrix H.
% Note: the numbering for n starts at 0 so the actual size of c is
% 4*(n+1)*m x 1

% Time constraints
t = [0 1 2 3];

% Waypoint constraints
% X axis
w(:, :, 1) = [  0   1   1   0; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints
            
% Y axis
w(:, :, 2) = [  0   0   2   2; ...
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
            
% Yaw
w(:, :, 4) = [  0   0   0   0; ...
                0   Inf Inf 0; ...  % angular velocity constraints
                0   Inf Inf 0; ...  % angular acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints

n_coeffs = n + 1;
H = zeros(n_coeffs * m, n_coeffs * m, states);
s = size(w);
n_constraints = s(1) * s(2);
% Aeq = zeros(n_constraints, n_coeffs * m, states);
% beq = zeros(n_constraints, 1, states);

for i = 1:states
    H(:,:,i) = buildh(n, m, mu_r, k_r, t);
    [Aeq{i}, beq{i}] = buildConstraints(n, k_r, w(:,:,i), t);
end

solution = zeros(n_coeffs * m, states); 
options = optimoptions('quadprog', 'Display', 'iter', 'MaxIterations', 4000);
for i = 1:states
    solution(:, i) = quadprog(H(:,:,i), [], [], [], Aeq{i}, beq{i}, [], [], [], options);
end

traj = discretizeTrajectory(solution, n, m, states, 0.01, t);

figure;
plot(traj(:,1), traj(:,2));








