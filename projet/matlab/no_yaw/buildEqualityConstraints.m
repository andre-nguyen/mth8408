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

n_states = 3;                           % x y z, no yaw
n_constraints = 2 * (m+1) * n_states;   % 2 per waypoint + 1 for initial conditions
% + 1 for final conditions
n_coeffs = n + 1;
ct_size = n_states * n_coeffs;  % size of a constraint in a row of A
coeffs = getCoefficientMatrix(n, k_r);
I = rot90(eye(n_coeffs, n_coeffs));


% First build fixed constraints
%A_0 = zeros(n_constraints, n_states * m * n_coeffs);
%b_0 = zeros(n_constraints);
A_0 = [];
b_0 = [];

% Each row of A is a constraint
for wp = 1:m+1                  % For each waypoint
    for der = 1:k_r               % For each derivative of the polynomial up to r-1
        for state = 1:n_states  % For each state x y z
            if w(der, wp, state) ~= Inf % We have a constraint
                if wp == 1
                    % Initial conditions
                    % Add only departure constraints
                    a = zeros(1, n_states * m * n_coeffs);
                    % Set the part of 'a' corresponding to the wp
                    %   XXX
                    int_t = 1 / (t(wp+1) - t(wp))^(der-1);  % first row is 0th derivative
                    poly = coeffs(der, :) .* I(der, :) * int_t;
                    %   Get the start index of a block
                    idx = (wp-1) * ct_size + (state-1) * n_coeffs + 1;
                    a(1, idx:idx+n) = poly;
                    b = w(der, wp, state);
                    
                elseif wp == m+1
                    % Final conditions
                    % Add only arrival constraints
                    a = zeros(1, n_states * m * n_coeffs);
                    % Set the part of 'a' corresponding to the wp
                    %   XXX
                    int_t_next = 1 / (t(wp) - t(wp-1))^(der-1);  % time now and prev
                    poly = coeffs(der, :) * int_t_next;
                    %   Get the start index of the "previous" block
                    idx = (wp-2) * ct_size + (state-1) * n_coeffs + 1;
                    a(1, idx:idx+n) = poly;
                    b = w(der, wp, state);
                else
                    % Middle/waypoint conditions
                    a = zeros(2, n_states * m * n_coeffs);
                    b = ones(2, 1);
                    % Add departure constraint
                    int_t_next = 1 / (t(wp) - t(wp-1))^(der-1);  % time now and prev
                    poly = coeffs(der, :) * int_t_next;
                    idx = (wp-2) * ct_size + (state-1) * n_coeffs + 1;
                    a(1, idx:idx+n) = poly;
                    
                    % Add arrival constraint
                    int_t = 1 / (t(wp+1) - t(wp))^(der-1);  % first row is 0th derivative
                    poly = coeffs(der, :) .* I(der, :) * int_t;
                    idx = (wp-1) * ct_size + (state-1) * n_coeffs + 1;
                    a(2, idx:idx+n) = poly;
                    
                    b = b .* w(der, wp, state); % both lines equal to this
                end
            end
            A_0 = [A_0; a];
            b_0 = [b_0; b];
        end
    end
end

% Now build continuity constraints follow equation 3.53 of "Control, 
% estimation, and planning algorithms for aggressive flight using onboard 
% sensing" By Bry, Adam Parker

% Basically what we will do is add arrival and derparture constraints on
% the same line to enforce continuity between two polynomials
A_t = [];
b_t = [];

for wp = 2:m        % XXX for each INTERMEDIATE waypoint
    for der = 1:k_r % for each dervative including the 0th derivative
        for state = 1:n_states % for each state x y z
            a = zeros(1, n_states * m * n_coeffs);
            int_t = 1 / (t(wp) - t(wp-1))^(der-1);
            int_t_next = 1 / (t(wp) - t(wp-1))^(der-1);
            
            % from prev wp
            a_prev = - coeffs(der, :) * int_t;  % note the minus
            idx_prev = (wp-2) * ct_size + (state-1) * n_coeffs + 1;
            a(1, idx_prev:idx_prev+n) = a_prev;
            
            % to next wp
            a_next = coeffs(der, :) .* I(der,:) * int_t_next;
            idx_next = (wp-1) * ct_size + (state-1) * n_coeffs + 1;
            a(1, idx_next:idx_next+n) = a_next;
            
            b = 0;
        end
        A_t = [A_t; a];
        b_t = [b_t; b];
    end
end

Aeq = [A_0; A_t];
beq = [b_0; b_t];

end
