k_r = 4;
k_psi = 2;
n = 6;
m = 3;

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
            
[A_eq, b_eq] = buildEqualityConstraints(n, m, k_r, k_psi, t, w);