%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fichier: model_problems.m
% Titre: Solution aux exercices de modélisation
% Auteur: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
close all;

[X1,X2] = meshgrid(-20:2:20);
Z = 100 .* (X2 - X1.^2).^2 + (1 - X1).^2;
figure
surf(X1, X2, Z);