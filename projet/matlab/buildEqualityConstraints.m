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

% Page 3 of http://rss2012_workshop.visual-navigation.com/pdf/richter_rss13_workshop.pdf
num_coeffs = n+1;       % number of coefficients in a polynomial
num_states = 4;         % x y z and psi
I = eye(num_coeffs);    % Identity matrix for compulations

% Build A_0 and b_0, the constraints relating to the waypoints and its
% derivatives,
A_0 = zeros(2*num_states*m, num_states*num_coeffs*m);
b_0 = zeros(2*num_states*m, 1);

for i = 1:m % For each position waypoint (basically the 0th derivative)
    % "A is constructed by evaluating the component of the derivative in
    % equation 9"
    waypoint = w(1, i, :);
    waypoint = reshape(waypoint, 4, 1);
    values = zeros(1, num_coeffs);
    if i == 1 % Initial conditions
        % Add only one constraint
        % Evaluate the polynomial
        for j=1:num_coeffs
            % Evaluate the polynomial at time t
            values(j) = polyval(I(j,:),t(i));
        end
        
        % Shove the polynomial values into A_0
        for j = 1:num_states
            a = zeros(1, num_states * num_coeffs * m);
            lower_idx =  (i-1) * num_coeffs * num_states + (j-1) * num_coeffs +1;
            upper_idx = ((i-1) * num_coeffs * num_states + (j-1) * num_coeffs   ) + num_coeffs;
            a(lower_idx : upper_idx) = values;
            A_0(j, :) = a;
        end
        b_0(1:num_states) = waypoint;
        
    else % Actual waypoints
        for j = 1:num_coeffs
            values(j) = polyval(I(j,:), t(i));
        end
        
        % We add two constraints, one at the beginning and one and the end
        % of a trajectory segment
        for j = 1:num_states
            a = zeros(1, num_states * num_coeffs * m);
            lower_idx =  (i-2) * num_coeffs * num_states + (j-1) * num_coeffs +1;
            upper_idx = ((i-2) * num_coeffs * num_states + (j-1) * num_coeffs   ) +num_coeffs;
            a(lower_idx : upper_idx) = values;
            A_0(j + 2 * num_states * (i-1), :) = a;
        end
        b_0(2 * num_states * (i-1) + 1 : ...
            2 * num_states * (i-1) + num_states) = waypoint;
        
        for j = 1:num_states
            a = zeros(1, num_states * num_coeffs * m);
            lower_idx =  (i-1) * num_coeffs * num_states + (j-1) * num_coeffs +1;
            upper_idx = ((i-1) * num_coeffs * num_states + (j-1) * num_coeffs   ) +num_coeffs;
            a(lower_idx : upper_idx) = values;
            A_0(j + 2 * num_states * (i-1) + num_states, :) = a;
        end
        b_0(2 * num_states * (i-1) + num_states + 1: ...
            2 * num_states * (i-1) + 2*num_states) = waypoint;
    end
end

% Derivative constraints (without yaw)
A_T = zeros(2 * m * (num_states-1) * k_r, num_states * num_coeffs * m);
b_T = ones(2 * m * (num_states-1) * k_r, 1) * eps;

for i = 1:m     % For each waypoint
    for j = 1:k_r   % For each derivative of the position states
        % speed, acceleration, jerk, snap
        % Evaluate all the polynomials for each derivative
        values = zeros(1, num_coeffs);
        for k = 1:num_coeffs
            tempCoeffs = I(j,:);
            for h = 1:j     % for 1 to the current derivative
                % Differentiate
                tempCoeffs = polyder(tempCoeffs);
            end
            % Evaluate the resulting polynomial at time t
            values(k) = polyval(tempCoeffs, t(i));
        end
        if i == 1
            % For the initial conditions
            for k = 1:num_states-1  % For states x y z (no yaw)
                a = zeros(1, num_states * num_coeffs * m);
                % Note
                % w rows: derivatives
                %   cols: waypoints
                %   page: state (x y z yaw)
                constraint = w(j+1, i, k);
                if constraint == Inf
                    % No set value, just enforce continuity. Usually we
                    % shouldn't get here since we want to be at rest when
                    % starting
                    lower_idx =  (i-1) * num_coeffs * num_states + (k-1) * num_coeffs + 1;
                    upper_idx = ((i-1) * num_coeffs * num_states + (k-1) * num_coeffs    ) + num_coeffs;
                    a(lower_idx : upper_idx) = values;
                    lower_idx =  (m-1) * num_coeffs * num_states + (k-1) * num_coeffs + 1;
                    upper_idx = ((m-1) * num_coeffs * num_states + (k-1) * num_coeffs    ) + num_coeffs;
                    a(lower_idx : upper_idx) = -values;
                    A_T(k + (j-1) * (num_states-1), :) = a;
                    b_T(k + (j-1) * (num_states-1)) = 0;
                else
                    % We have a constraint on the derivative so set the
                    % polynomial equal to that.
                    lower_idx =  (i-1) * num_coeffs * num_states + (k-1) * num_coeffs + 1;
                    upper_idx = ((i-1) * num_coeffs * num_states + (k-1) * num_coeffs    ) + num_coeffs;
                    a(lower_idx : upper_idx) = values;
                    A_T(k + (j-1) * (num_states-1), :) = a;
                    b_T(k + (j-1) * (num_states-1)) = constraint;
                end
            end
            
        else
            % For all other waypoints
            for k = 1:num_states-1
                a = zeros(1, num_states * num_coeffs * m);
                constraint = w(j+1, i, k);
                if constraint == Inf
                    % No constraint, just enforce continuity
                    lower_idx =  (i-2) * num_coeffs * num_states + (k-1) * num_coeffs +1;
                    upper_idx = ((i-2) * num_coeffs * num_states + (k-1) * num_coeffs   ) + num_coeffs;
                    a(lower_idx : upper_idx) = values;
                    lower_idx =  (i-1) * num_coeffs * num_states + (k-1) * num_coeffs +1;
                    upper_idx = ((i-1) * num_coeffs * num_states + (k-1) * num_coeffs   ) + num_coeffs;
                    a(lower_idx : upper_idx) = -values;
                    A_T(k + (j-1) * (num_states-1) + 2 * (i-1) * (num_states-1) * k_r, :) = a;
                    b_T(k + (j-1) * (num_states-1) + 2 * (i-1) * (num_states-1) * k_r) = 0;
                else
                    % We have a constraint
                    lower_idx =  (i-2) * num_coeffs * num_states + (k-1) * num_coeffs +1;
                    upper_idx = ((i-2) * num_coeffs * num_states + (k-1) * num_coeffs   ) + num_coeffs;
                    a(lower_idx : upper_idx) = values;
                    A_T(k + (j-1) * (num_states-1) + 2 * (i-1) * (num_states-1) * k_r, :) = c;
                    b_T(k + (j-1) * (num_states-1) + 2 * (i-1) * (num_states-1) * k_r) = constraint;
                end
            end
        end
    end
end

% Equation (17)
Aeq = [A_0; A_T];
beq = [b_0; b_T];
end

