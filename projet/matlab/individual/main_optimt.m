addpath('helpers');
setupProblem();

global t0;

T0 = time2segment(t0);

Aeq = ones(1, length(T0));
beq = t0(end);
lb = zeros(1, length(T0));

tic
global save_intermediate_solutions;
if save_intermediate_solutions
    options = optimoptions('fmincon', ...
                            'SpecifyObjectiveGradient',true, ...
                            'Display', 'iter-detailed', ...
                            'OutputFcn', @savetrajfun);
else
    options = optimoptions('fmincon', ...
                            'SpecifyObjectiveGradient',true, ...
                            'Display', 'iter-detailed');
end
T = fmincon(@objective, T0, [], [], Aeq, beq, lb, [], [], options);
t_final = segment2time(T);
toc

%% Plot solutions
global solutions;
global m;
iters = length(solutions);
set(0,'DefaultAxesColorOrder', parula(iters));
close all;
figure
hold on;
handles = [];
for i = 1:iters
    x = solutions{i}.discrete(:,1);
    y = solutions{i}.discrete(:,2);
    l = length(x) / m;
%     plot(x(1:l), y(1:l), '-r');
%     plot(x(l+1:2*l), y(l+1:2*l), '-g');
%     plot(x(2*l+1:3*l), y(2*l+1:3*l), '-b');
    if i == 1
        h(i) = plot(x, y, '--');
    elseif i == iters
        h(i) = plot(x, y, '.');
    else
        h(i) = plot(x,y);
    end
end
legend(h([1 iters]), 'initial guess', 'final solution');
grid on;
axis equal;