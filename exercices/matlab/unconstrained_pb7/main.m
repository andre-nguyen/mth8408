%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   main.m
%   problem 7
%   Author: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = [1.5;0.5];

fprintf('Steepest descent on f_1\n');
[sol1, conv1] = steepest(x0, @obj1, true);

fprintf('\n\n Newton''s method on f_1\n');
[sol2, conv2] = newton(x0, @obj1, true);


fprintf('\n\nSteepest descent on f_2\n');
[sol, conv3] = steepest(x0, @obj2, true);

fprintf('\n\n Newton''s method on f_2\n');
[sol, conv4] = newton(x0, @obj2, true);

close all;
figure;
x1 = 1:length(conv1);
x2 = 1:length(conv2);
x3 = 1:length(conv3);
x4 = 1:length(conv4);

subplot(1,2, 1);
plot(x1, conv1, x2, conv2);
grid on;
legend('Steepest descent', 'Modified newtons');
title('Value of f_1 at each iteration');
subplot(1,2,2);
plot(x3, conv3, x4, conv4);
grid on;
legend('Steepest descent', 'Modified newtons');
title('Value of f_2 at each iteration');

% draw contour plot for f_1
x = linspace(0,2);
y = linspace(-1,1);
[X,Y] = meshgrid(x,y);
Z = 2.*X.^3 - 3.*X.^2 - 6.*X.*Y.*(X-Y-1);
figure
subplot(1,2,1);
contour(X,Y,Z,'ShowText','on');
xlabel('x');
ylabel('y');
hold on;
plot(sol1(1,:), sol1(2,:), 'r--');
hold on;
scatter(1.5, 0.5, 'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5);
hold on;
scatter(sol1(1,end), sol1(2,end), 'd', 'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5);
legend('contour', 'path', 'start', 'end')
title('Iterations on f_1 using steepest descent');
grid on;

subplot(1,2,2);
contour(X,Y,Z,'ShowText','on');
xlabel('x');
ylabel('y');
hold on;
plot(sol2(1,:), sol2(2,:), 'r--');
hold on;
scatter(1.5, 0.5, 'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5);
hold on;
scatter(sol2(1,end), sol2(2,end), 'd', 'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5);
legend('contour', 'path', 'start', 'end')
title('Iterations on f_1 using newton''s');
grid on;