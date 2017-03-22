function [ traj ] = discretizeTrajectory( coeffs, n, m, states, dt, t)
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


n_coeffs = n + 1;
ct_size = n_coeffs * states;
n_total_samples = (max(t) / dt);
traj = zeros(n_total_samples, states);

for wp = 1:m
    segment_samples = (t(wp+1) - t(wp)) / dt;
    lower_idx = (wp-1) * segment_samples + 1;
    upper_idx = wp * segment_samples;
    for state = 1:states
        l_coeffs_idx = (wp-1) * n_coeffs + 1;
        h_coeffs_idx = l_coeffs_idx + n_coeffs - 1;
        segment_coeffs = coeffs(l_coeffs_idx:h_coeffs_idx, state);
        traj(lower_idx:upper_idx, state) = polyval(segment_coeffs, t(wp):dt:t(wp+1)-dt);
    end
end
end
