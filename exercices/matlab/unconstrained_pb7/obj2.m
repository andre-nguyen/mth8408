function [f, g, H] = obj2(x)
% obj2 Second objective function
%   
% Inputs
%   x   Point at which to evaluate the function
% Output 
%   f   Result of the function evaluated at x
%   g   Result of the gradient evaluated at x
%   H   Result of the Hessian evaluated at x

f = 0;
for i = 1:length(x)-1
    tmp = 100 * (x(i+1) - x(i)^2)^2 + (1-x(i))^2;
    f = f + tmp;
end