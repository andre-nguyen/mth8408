addpath('helpers');

t0 = [0 0.5 2.5 3];

T0 = time2segment(t0);

Aeq = ones(1, length(T0));
beq = t0(end);
lb = zeros(1, length(T0));

tic
options = optimoptions('fmincon', ...
                        'SpecifyObjectiveGradient',true, ...
                        'Display', 'iter', ...
                        'PlotFcn', @optimplotfval);
T = fmincon(@objective, T0, [], [], Aeq, beq, lb, [], [], options);
toc