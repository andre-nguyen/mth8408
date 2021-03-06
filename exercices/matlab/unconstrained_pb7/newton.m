function [x_all, convergence] = newton(x0, obj, verbose)
% newton Newton descent method with armijo linesearch for problem 7
%   
% Inputs
%   x0  Point at which to start
%   obj Objective function we want to optimize
%   verbose Print out the iterations
% Output 
%   x   End point
% Author: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

if nargin > 3
    error('steepest too many inputs')
end

switch nargin
    case 2
        verbose = false;
end

epsilon_a = 0.001;
epsilon_r = 0.001;
max_iter  = 20;
x = x0;
[~, gxk, ~] = obj(x);   % gxk gradient at x_k
[~, gx0, ~] = obj(x0);  % gx0 gradient at x_0

if verbose
    fprintf('k\tf(x)\t\tt_k\tx1\tx2\n');
end

i = 0;
convergence = [];
x_all = [];
while   (norm(gxk) > epsilon_a + epsilon_r*norm(gx0)) && (i < max_iter)
    [f, gxk, Hx] = obj(x);
    d = -(Hx \ gxk);
    % verify its a descent direction
    slope = gxk' * d;
    if slope > 0
        eig_vals = eig(Hx);
        lambda = min(eig_vals);
        lambda = lambda + 0.1;  % la petite constante
        Hx = Hx + lambda * eye(length(Hx));
        d = Hx \ -gxk;
    end
    tk = armijo(x, d, obj);
    
    if verbose
        g=sprintf('%2.2f ', x);
        fprintf('%d\t %4.4f\t %1.4f\t %s\n', i, f, tk, g);
    end
    
    convergence = [convergence f];
    x_all = [x_all x];
    x = x + tk * d;
    i = i + 1;
end
