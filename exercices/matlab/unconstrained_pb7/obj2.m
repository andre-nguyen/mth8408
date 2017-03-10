function [f, g, H] = obj2(x)
% obj2 Second objective function if problem 7
%   
% Inputs
%   x   Point at which to evaluate the function
% Output 
%   f   Result of the function evaluated at x
%   g   Result of the gradient evaluated at x
%   H   Result of the Hessian evaluated at x
% Author: Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

f = 0;
n = length(x) - 1;
for i = 1:n
    tmp = 100 * (x(i+1) - x(i)^2)^2 + (1-x(i))^2;
    f = f + tmp;
end

g = zeros(n, 1);
for i = 1:n
    g(i) = -400 * x(i) * (x(i+1)-x(i)^2) + 2 * (1-x(i));
end


H = zeros(n, n);
for i = 1:n
    % iterate only on the diagonal
    H(i,i) = -400 * x(i+1) + 1200 * x(i)^2 -2;
    if i ~= n
        % Last row doesn't have this term
        H(i,i+1) = -400 * x(i);
    end        
end