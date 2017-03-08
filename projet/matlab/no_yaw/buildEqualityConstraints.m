function [ Aeq, beq ] = buildEqualityConstraints( n, m, k_r, k_psi, t, w)
%BUILDEQUALITYCONSTRAINTS Builds the the equality constraint matrices to be
%fed to quadprog
% Inputs:
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints
%   k_r         Order of the derivative of the position
%   k_psi       Order of the derivative of the yaw angle
%   t           Vector of the arrival times for each waypoint. Should
%               always 0 as the first element.
%   w           Waypoints, in the paper they call them keyframes. Each row
%               is a waypoint with the data in this order: x y z psi vx vy
%               vz vpsi ax ay az apsi

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

% Mathematical basis
% Recall the form of the quadratic optimization problem to be
%   min c'Hc + f'c
% Where c is an 4mn x 1 vector containing the coefficients of all the
% polynomials describing each dimension of the trajectory. In this function
% we are building the equality matrix constraints where
%   Aeq * c = beq
% Where Aeq is a matrix of doubles and beq is a vector of doubles

n_states = 3;       % x y z, no yaw
n_constraints = 0;  % 2 per waypoint + 1 for initial conditions + 1 for final conditions
n_coeffs = n + 1;
I = eye(n_coeffs);
% First build fixed constraints
A_0 = zeros(n_constraints, n_states * m * n_coeffs);
b_0 = zeros(n_constraints);

% Each row of A is a constraint
for wp = 0:m                  % For each waypoint
  for der = 0:r-1             % For each derivative of the polynomial up to r-1
      for state = 1:n_states  % For each state x y z
        if w(der+1, wp+1, state) ~= Inf % We have a constraint
          if wp == 0
            % Initial conditions
            % Add only 1 constraint
            a = zeros(1, n_states * m * n_coeffs);
            % Set the part of a corresponding to the wp
          elseif wp == m
            % Final conditions

          else
            % Middle/waypoitn conditions

          end
        end
      end
  end
end

end
