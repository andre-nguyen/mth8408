%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fichier: model_problems.m
% Titre: Solution aux exercices de modélisation
% Auteur: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;

%% problem 2
[X1,X2] = meshgrid(-2:0.1:2);
Z = 100 .* (X2 - X1.^2).^2 + (1 - X1).^2;

figure
subplot(1, 2, 1)
surf(X1, X2, Z);
xlabel('x1')
ylabel('x2')
zlabel('f(x1, x2)')
title('Problem 2')

subplot(1, 2, 2)
contour(X1, X2, Z, 10)
xlabel('x1')
ylabel('x2')
title('Contour plot')