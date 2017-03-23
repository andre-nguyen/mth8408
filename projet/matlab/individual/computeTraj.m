function [ traj, total_cost ] = computeTraj( n, m, states, k_r, mu_r, t, w )
%COMPUTETRAJ Summary of this function goes here
%   Detailed explanation goes here

n_coeffs = n + 1;
H = zeros(n_coeffs * m, n_coeffs * m, states);
s = size(w);

for i = 1:states
    H(:,:,i) = buildh(n, m, mu_r, k_r, t);
    [Aeq{i}, beq{i}] = buildConstraints(n, k_r, w(:,:,i), t);
end

solution = zeros(n_coeffs * m, states); 
options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 4000);
total_cost = 0;
for i = 1:states
    [solution(:, i), fval] = quadprog(H(:,:,i), [], [], [], Aeq{i}, beq{i}, [], [], [], options);
    %fprintf('fval: %d \n', fval);
    total_cost = total_cost + fval;
end
traj = discretizeTrajectory(solution, n, m, states, 0.01, t, false);

end

