addpath('helpers');

t = [0 0.5 2.5 3];

% Optimization stuff
h = 0.1;            
max_iters = 10;
gi = eye(m);
gi(gi==0) = (-1/(m-1));

% initial t is like our x0
[bestsol, prevcost] = computeTraj(n, m, states, k_r, mu_r, t, w);
sols = zeros([size(bestsol) m]);
costs = zeros(1, m);
k = 0;
while k < max_iters
    grad_gi_f = zeros(m, 1);
    for i = 1:m
        T = time2segment(t);
        T = T + (h * gi(:, i));
        t = segment2time(t);
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
