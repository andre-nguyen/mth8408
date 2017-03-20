function [ Aeq, beq ] = buildConstraints( n, r, constraints, t)
%BUILDCONSTRAINTS Builds the constraints matrix
% Inputs:
%   n           Order of the polynomials of a trajectory
%   r           Order of the derivative of the position
%   constraints r by wps+1 Matrix of constraints
%               Rows are derivatives and columns are waypoints
%               Inf denotes unfixed variable.
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% 
% Basically follow equations 16 to 21 of richter_rss13_workshop
num_coeffs = n+1;
s = size(constraints);
num_wps = s(2);

Aeq = [];
beq = [];

for wp = 1:num_wps
    for der = 0:r-1
        if constraints(der+1, wp) ~= Inf
            A_0 = zeros(2, num_coeffs);
            b_0 = zeros(2, 1);
            if wp ~= num_wps
                % eq 3.34 Segment start constraint
                cum_mul = 1;
                for i = 0:r-1   % i is m in the eq
                    cum_mul = cum_mul * (r-i);
                end
                A_0(1, der+1) = cum_mul;
                b_0(1) = constraints(der+1,wp);  % eq 3.35
                
                % eq 3.36 Segment end constraint
                for c = 0:num_coeffs-1
                    if c >= der
                        tau = t(wp+1);
                        cum_mul = 1;
                        for i = 0:r-1
                            cum_mul = cum_mul * (c-i) * tau^(c-der);
                        end
                    end
                end
                A_0(2, der+1) = cum_mul;
                b_0(2) = constraints(der+1,wp);  % eq 3.37
            else
                
            end
            
            Aeq = [Aeq; A_0];
            beq = [beq; b_0];
        end
    end
end


end

