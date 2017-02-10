function [ H ] = buildh( n, m, mu_r, mu_psi, k_r, k_psi, t )
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

assert(m == length(t)); % Check if the number of arrival times matches the 
                        % number of waypoints

num_coeffs = n + 1;

% Differentiate the polynomials the required number of times
poly_coeffs_r = ones(1, num_coeffs );
for i = 1:k_r
    poly_coeffs_r = polyder(poly_coeffs_r);
end
% Pad with zeros
poly_coeffs_r = [poly_coeffs_r zeros(1, num_coeffs - length(poly_coeffs_r))];

poly_coeffs_psi = ones(1, num_coeffs );
for i = 1:k_psi
    poly_coeffs_psi = polyder(poly_coeffs_psi);
end

oit = 2*(n - k_r) + 1;  % Find highest order of the integrated t
oit = oit:-1:1;         % All exponents down to 1
oit = [oit zeros(1,num_coeffs-length(oit))]; % Pad with 0

% For each polynomial between two waypoints, build the four H_x, H_y, H_z
% and H_psi matrices and then concatenate them diagonally to H.
H = [];
for i = 1:m-1
    H_x = zeros(num_coeffs, num_coeffs);
    H_y = zeros(num_coeffs, num_coeffs);
    H_z = zeros(num_coeffs, num_coeffs);
    H_psi = zeros(num_coeffs, num_coeffs);
    
    % Upper triangular iteration
    for j = 1:num_coeffs
        for k = j:num_coeffs
            if j == k
                H_x(j,k) = poly_coefs_r(j)^2 * ...
                    (1/oit(k)) *  (t(i+1)-t(i))^(oit(k)); % integral
            end
            
        end
    end
    
    
end

end
