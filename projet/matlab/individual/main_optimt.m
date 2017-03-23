% find optimal segment times through gradient descent
k_r = 4;    % Order of derivative of the position
mu_r = 1;
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
states = 3;

t = [0 0.5 2.5 3];

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

% Optimization stuff
h = 0.1;            
max_iters = 10;
gi = eye(m);
gi(gi==0) = (-1/(m-1));
% initial t is like our x0
[bestsol, prevcost] = computeTraj(n, m, states, k_r, mu_r, t, w);
sols = zeros([size(bestsol) m];
costs = zeros(1, m);
k = 0;
while k < max_iters
    grad_gi_f = zeros(m, 1);
    for i = 1:m
        T = (t(2:end) - t(1:end-1))';
        T = T + (h * gi(:, i));
        t = [0 cumsum(T')];
        [sols(:,:,i), costs(i)] = computeTraj(n, m, states, k_r, mu_r, t, w);
        grad_gi_f(i) = (costs(i) - prevcost) / h;
    end
    
    [~, idx] = min(grad_gi_f);
    T = T + (h * gi(:, i));
    t = [0 cumsum(T')];

    if cost < prevcost
        bestsol = sol;
    end
    k = k + 1;
end
