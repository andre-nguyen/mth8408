function [ cost, varargout ] = objective(t, h)
addpath(helpers);
% [cost]
% [cost, gradient]

% find optimal segment times through gradient descent
k_r = 4;    % Order of derivative of the position
mu_r = 1;
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
states = 3;

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

[~, cost] = computeTraj(n, m, states, k_r, mu_r, t, w);

% Additional args
n_addargs = max(nargout,1) - 1;
if n_addargs == 1
    % compute gradient
    gi = eye(m);                % m can also be seen as the number of segments
    gi(gi==0) = -1 / (m+1-2);   % m+1 = num keyframes
    nabla_gi_f = zeros(m, 1);
    for i = 1:m
        T = time2segment(t);
        T = T + h * gi(:,i);
        t = segment2time(T);
        [~, cost_Thgi] = computeTraj(n, m, states, k_r, mu_r, t, w);
        nabla_gi_f(i) = (cost_Thgi - cost) / h;
    end
    varargout{1} = nabla_gi_f;    
end

end

