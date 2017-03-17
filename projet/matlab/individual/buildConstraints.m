function [ A, b ] = buildConstraints( n, wps, r, constraints, t)
%BUILDCONSTRAINTS Builds the constraints matrix
% Inputs:
%   n           Order of the polynomials of a trajectory
%   wps         Number of waypoints (not including initial conditions)
%   r           Order of the derivative of the position
%   constraints r by wps+1 Matrix of constraints
%               Rows are derivatives and columns are waypoints
%               Inf denotes unfixed variable.
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% 
% Basically follow equations 16 to 21 of richter_rss13_workshop
num_coeffs = n+1;
A_0 = zeros(num_coeffs);
A_tau = A_0;

for i = 1:num_coeffs
     % diagonal iteration
     cum_mul = 1;
     for j = 0:r-1
         cum_mul = cum_mul * (r-j);
     end
     
     A_0(i,i) = cum_mul;
end

end

