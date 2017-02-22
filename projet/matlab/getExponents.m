function [ exp_r, exp_psi ] = getExponents( n, k_r, k_psi )
%GETEXPONENTS Generates the exponents (order) required to build the H matrix
% Inputs:
%   n           Order of the polynomials of a trajectory
%   k_r         Order of the derivative of the position
%   k_psi       Order of the derivative of the yaw angle
%
% Outputs:
%   exp_r       Exponents for the r polynomials
%   exp_psi     Exponents for the psi polynomial

num_coeffs = n + 1;

tmp = n - k_r;  % Gives the largest exponent after derivation
tmp = tmp * 2;  % Square the norm (eq 5) so double the exponent
tmp = tmp + 1;  % Integrating (eq 6) adds 1 to the exponent
exp_r = tmp:-1:1;

tmp = n - k_psi;    % largest exponent after derivation
tmp = tmp * 2;      % square the polynomial so double the exponent
tmp = tmp + 1;      % Integrating adds 1 to the exponent
exp_psi = tmp:-1:1;

% Adjust the outputs so as to match the size of the H matrix
if length(exp_r) > num_coeffs
    % too big truncate
    exp_r = exp_r(1:num_coeffs);
elseif length(exp_r) < num_coeffs
    % too small pad with ones (mathematically it should be zero but we pad
    % with ones to prevent a division by zero later)
    exp_r = [exp_r ones(1, num_coeffs-length(exp_r))];
end

if length(exp_psi) > num_coeffs
    exp_psi = exp_psi(1:num_coeffs);
elseif length(exp_psi) < num_coeffs
    exp_psi = [exp_psi ones(1, num_coeffs-length(exp_psi))];
end

end

