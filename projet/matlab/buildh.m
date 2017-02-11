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
poly_coeffs_psi = [poly_coeffs_psi zeros(1, num_coeffs - length(poly_coeffs_psi))];

% Vector oit contains all the exponents the the polynomial once it is
% derivated and integrated. For example for n = 6 and k_r = 4, oit will
% have the form [5 4 3 2 1 0 0]
oit_r = 2*(n - k_r) + 1;    % Find highest order of the integrated t
oit_r = oit_r:-1:1;         % All exponents down to 1
oit_r = [oit_r ones(1,num_coeffs-length(oit_r))]; % Pad with 1 (instead of zeros 
                                            % to prevent division by zero)

oit_psi = 2*(n - k_psi) + 1;
oit_psi = oit_psi:-1:1;
oit_psi = [oit_psi ones(1, num_coeffs-length(oit_psi))];

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
            % Max with one to avoid division by zero
            o_r = max((n-k_r-j+1)+(n-k_r-k+1) + 1, 1);
            o_psi = max((n-k_psi-j+1)+(n-k_psi-k+1) + 1, 1);
            if j == k
                H_x(j,k) = poly_coeffs_r(j)^2 * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r; % integral
                H_y(j,k) = poly_coeffs_r(j)^2 * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r; 
                H_z(j,k) = poly_coeffs_r(j)^2 * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r;
                
                H_psi(j,k) = poly_coeffs_psi(j)^2 * ...
                    (1/o_psi) * (t(i+1)-t(i))^o_psi;
            else
                H_x(j,k) = 2 * poly_coeffs_r(j) * poly_coeffs_r(k) * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r; % integral
                H_y(j,k) = 2 * poly_coeffs_r(j) * poly_coeffs_r(k) * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r;
                H_z(j,k) = 2 * poly_coeffs_r(j) * poly_coeffs_r(k) * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r;
                
                H_psi(j,k) = 2 * poly_coeffs_psi(j) * poly_coeffs_psi(k) * ...
                    (1/o_psi) * (t(i+1)-t(i))^o_psi;
            end            
        end
    end
end

end
