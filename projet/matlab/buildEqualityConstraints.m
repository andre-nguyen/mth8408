function [ Aeq, beq ] = buildEqualityConstraints( n, m, k_r, k_psi, t, w)
%BUILDEQUALITYCONSTRAINTS Builds the the equality constraint matrices to be
%fed to quadprog
% Inputs:
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints
%   k_r         Order of the derivative of the position
%   k_psi       Order of the derivative of the yaw angle
%   t           Vector of the arrival times for each waypoint. Should
%               always 0 as the first element.
%   w           Waypoints, in the paper they call them keyframes. Each row
%               is a waypoint with the data in this order: x y z psi vx vy
%               vz vpsi ax ay az apsi

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

% Mathematical basis
% Recall the form of the quadratic optimization problem to be
%   min c'Hc + f'c
% Where c is an 4mn x 1 vector containing the coefficients of all the
% polynomials describing each dimention of the trajectory. In this function
% we are building the equality matrix constraints where
%   Aeq * c = beq 
% Where Aeq is a matrix of doubles and beq is a vector of doubles

% Page 3 of http://rss2012_workshop.visual-navigation.com/pdf/richter_rss13_workshop.pdf

% Build A_0
A_0 = zeros([k_r+1 n+1]);
for i = 0:1:k_r+1
   for j = 0:1:n+1
       if i == j
           tmp = zeros([1 k_r]);
           for k = 0:1:k_r-1
               tmp(k+1) = (k_r-k); % eq 18
           end
           A_0(i+1,j+1) = prod(tmp);
       end           
   end
end

% Build b_0

end

