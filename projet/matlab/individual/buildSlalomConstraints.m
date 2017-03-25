function [ A, b ] = buildSlalomConstraints( n, m, r, vwp, epsilon, t, delta)
%BUILDSLALOMCONSTRAINTS Builds the inequality constraints matrix for the
%virtual waypoints of a slalom trajectory
% INPUTS
%  n        Order of the polynomials of the trajectory
%  m        Number of waypoints (excluding initial conditions)
%  r        Oder of the derivative of the position
%  vwp      Virtual waypoints
%             Column vector for each virtual waypoint in this dimension
%  epsilon  Scalar, min distance from virtual waypoint. Has a direction!
%  t        Time vector for each virtual waypoint
%  delta    Deltatime before and after time t where the distance epsilon
%           has to be respected

assert(isscalar(epsilon));      % epsilon is scalar
assert(isscalar(delta));        % delta is scalar
vwp_size = size(vwp);
assert(vwp_size(2) == 1);       % vwp is a column vector

n_coeffs = n+1;
coeffs = getCoefficientMatrix(n, r);

for wp = 1:length(vwp)
    a = zeros(1, m * n_coeffs);
    int_t = 1 / (t(wp) - t(wp))^(1);
end

end

