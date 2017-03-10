function [t] = armijo(x, d, obj)
% armijo Armijo line search for problem 7
%   
% Inputs
%   x   Point at which to do the line search
%   d   Direction in which to do the line search
%   obj Objective function we want to optimize
% Output 
%   t   Step length from 0 to 1
% Author: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%
% Theory
% See slides 19/71 take a big step length and shorten it until the armijo 
% condition is satisfied. The Armijo condition is basically that the new 
% step length should make us go somewhere where the function value is 
% smaller than if we had taken a full step.

% full step value
t = 1;
alpha = 0.5;
[armijo, ~] = obj(x + t*d);

% right hand side
[fx, gradx, ~] = obj(x);
rhs = fx + alpha * t * gradx' * d;

while (armijo > rhs) && (t > 10*eps) 
    t = t/2;
    rhs = fx + alpha * t * gradx' * d;
    [armijo, ~] = obj(x + t*d);
end

end