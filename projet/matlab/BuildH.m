function [ H ] = BuildH( n, m, mu_r, mu_psi, k_r, k_psi, t )
%BUILDH Builds the H matrix for the QP problem for the minimum snap
%trajectory generation.
% Inputs:
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints
%   mu_r        Constant making the position integrand non-dimentional
%   mu_psi      Cosntant making the yaw integrand non-dimentional
%   k_r         Order of the derivative of the position
%   k_psi       Order of the derivative of the yaw angle
%   t           Vector of the arrival times for each waypoint. Should
%               always 0 as the first element.
%
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

assert(m == length(t)); % Check if the number of times matches the number 
                        % of waypoints

num_coefficients = n + 1;
% Differentiate the polynomials the required number of times
poly_coefs_r = ones(1, num_coefficients );
for i = 1:k_r
    poly_coefs_r = polyder(poly_coefs_r);
end

poly_coefs_psi = ones(1, num_coefficients );
for i = 1:k_psi
    poly_coefs_psi = polyder(poly_coefs_psi);
end

% For each polynomial between two waypoints, build the four H_x, H_y, H_z
% and H_psi matrices and then concatenate them diagonally to H.
H = [];
for i = 1:m
    H_x = zeros(num_coefficients, num_coefficients);
    H_y = zeros(num_coefficients, num_coefficients);
    H_z = zeros(num_coefficients, num_coefficients);
    H_psi = zeros(num_coefficients, num_coefficients);
    % Upper triangular iteration and then diagonal mirror to make it
    % symmetric
    for j = 1:num_coefficients
        for k = j:num_coefficients
            if j == k
                H_x(i,j) = poly_coeffs_r(j)^2 * (1/4) *  (t(m+1)-t(m))^3
            end
        end
    end
end

end