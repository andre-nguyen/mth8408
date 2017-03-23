function [ traj ] = discretizeTrajectory( coeffs, n, m, states, dt, t, do_plot)
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

if nargin < 6
    do_plot = false
end
    

n_coeffs = n + 1;
traj = [];

if do_plot
    figure
    hold on
end
for wp = 1:m
    time = 0:dt:1;
    segment_samples = length(time);
    segment = zeros(segment_samples, states);
    for state = 1:states
        l_coeffs_idx = (wp-1) * n_coeffs + 1;
        h_coeffs_idx = l_coeffs_idx + n_coeffs - 1;
        segment_coeffs = coeffs(l_coeffs_idx:h_coeffs_idx, state);

        segment(:, state) = polyval(segment_coeffs, time);
    end
    if do_plot
        plot(segment(:, 1), segment(:, 2), 'o-');
        grid on
        grid minor
        axis equal
    end
    traj = [traj; segment];
end

end
