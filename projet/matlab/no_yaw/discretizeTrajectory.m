function [ traj ] = discretizeTrajectory( coeffs, n, m, states, dt, times)
%DISCRETIZETRAJECTORY Takes in the solution to the QP problem and
%discretizes it to plot the trajectory.
% Inputs:
%   coeffs      Solution to QP problem
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints
%   states      Number of states x, y, z and maybe psi
%   dt          Time step
%   times       Vector of the arrival times for each waypoint. Should
%               always 0 as the first element.

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

for t = 
for wp = 1:m
    for state = 1:states
        
    end 
end

end

