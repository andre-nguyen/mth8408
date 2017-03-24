function [ steplength ] = armijo( t, d, obj )
%ARMIJO Line search
% 
% Inputs
%  t    Arrival times used to do the linesearch
%  d    Direction in which to do the line sesarch
%  obj  Objective function we want to optimize
% Output
%  steplength   from 0 to 1

steplength = 1;
alpha = 1;
[armijo] = obj();

end

