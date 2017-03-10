function [f, g, H] = obj1(point)
% obj1 First objective function
%   f_1(x,y) = 2x^3 − 3x^2 − 6xy(x − y − 1)
% Inputs
%   x   Point at which to evaluate the function
% Output
%   f   Result of the function evaluated at x
%   g   Result of the gradient evaluated at x
%   H   Result of the Hessian evaluated at x

x = point(1);
y = point(2);

% function
f = 2*x^3 - 3*x^2 - 6*x*y*(x-y-1);

% gradient
g = [   6 * (x^2 -x -2*x*y +y^2 +y);
        6*x*(2*y - x + 1)];

% hessian
H = [   12*x-12*y-6,        12*y - 12*x + 6; ...
        12*y - 12*x + 6,    12*x];
