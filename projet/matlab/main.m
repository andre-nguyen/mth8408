%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Filename:   main.m
%   Author:     Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%   Class:      MTH8408
%   Description:Implementation of "Minimum Snap Trajectory Generation
%               and Control for Quadrotors"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

k_r = 4;    % Order of derivative of the position
k_psi = 2;  % Order of derivative of the yaw
mu_r = 1;   % Non-dimentionalization constant for the position integral
mu_psi = 1; % Non-dimentionalization constant for the yaw integral
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)

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
            
% Yaw
w(:, :, 4) = [  0   0   0   0; ...
                0   Inf Inf 0; ...  % angular velocity constraints
                0   Inf Inf 0; ...  % angular acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints

H = buildh(n, m, mu_r, mu_psi, k_r, k_psi, t);
[Aeq, beq] = buildEqualityConstraints(n, m, k_r, k_psi, t, w);