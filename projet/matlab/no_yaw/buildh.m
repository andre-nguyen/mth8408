function [ H ] = buildh( n, m, mu_r, mu_psi, k_r, k_psi, t )
%BUILDH Builds the H matrix for the QP problem for the minimum snap
%trajectory generation.
% Inputs:
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints (not including initial conditions)
%   mu_r        Constant making the position integrand non-dimentional
%   mu_psi      Cosntant making the yaw integrand non-dimentional
%   k_r         Order of the derivative of the position
%   k_psi       Order of the derivative of the yaw angle
%   t           Vector of the arrival times for each waypoint. Should
%               always have 0 as the first element followed by m arrival
%               times
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% Good refenrece paper
% http://rss2012_workshop.visual-navigation.com/pdf/richter_rss13_workshop.pdf
% The original minimum snap paper is a bit short on details but gets cited
% a lot because they make the proof that a quadrotor is differentially
% flat. But the paper by Ritcher is better for implementing.

assert(m == length(t)-1);   % Check if the number of arrival times matches
                            % the number of waypoints

num_coeffs = n + 1;

% Differentiate the polynomials the required number of times
poly_coeffs_r = ones(1, num_coeffs);
for i = 1:k_r
    poly_coeffs_r = polyder(poly_coeffs_r);
end
% Pad with zeros
poly_coeffs_r = [poly_coeffs_r zeros(1, num_coeffs - length(poly_coeffs_r))];

% For each polynomial between two waypoints, build the four H_x, H_y, H_z
% and H_psi matrices and then concatenate them diagonally to H.
H = [];
for i = 1:m
    H_x = zeros(num_coeffs, num_coeffs);
    H_y = zeros(num_coeffs, num_coeffs);
    H_z = zeros(num_coeffs, num_coeffs);
    [oit_r, oit_psi] = getExponents(n, k_r, k_psi);

    % Upper triangular iteration
    for j = 1:num_coeffs
        for k = j:num_coeffs
            idx = k - j + 1;        % start iteration at 1 at each row
            o_r = oit_r(idx);
            o_psi = oit_psi(idx);

            if j == k
                H_x(j,k) = poly_coeffs_r(j)^2 * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r; % integral
                H_y(j,k) = H_x(j,k);
                H_z(j,k) = H_x(j,k);

%                H_psi(j,k) = poly_coeffs_psi(j)^2 * ...
%                    (1/o_psi) * (t(i+1)-t(i))^o_psi;
            else
                H_x(j,k) = 2 * poly_coeffs_r(j) * poly_coeffs_r(k) * ...
                    (1/o_r) *  (t(i+1)-t(i))^o_r; % integral
                H_y(j,k) = H_x(j,k);
                H_z(j,k) = H_x(j,k);

%                H_psi(j,k) = 2 * poly_coeffs_psi(j) * poly_coeffs_psi(k) * ...
%                    (1/o_psi) * (t(i+1)-t(i))^o_psi;
            end
        end

        % Shift out the exponents and shift in the next exponents
        oit_r = [oit_r(3:end) oit_r(end)-1 oit_r(end)-2];
        oit_r(oit_r<1) = 1; % threshold to 1 to prevent division by 0
        oit_psi = [oit_psi(3:end) oit_psi(end)-1 oit_psi(end)-2];
        oit_psi(oit_psi<1) = 1;
    end
    % Finally get to the diagonal concatenation
    H = blkdiag(H, mu_r*H_x, mu_r*H_y, mu_r*H_z)%, mu_psi*H_psi);
end

% Make the H matrix symetric
H = H + H' - diag(diag(H));
end
