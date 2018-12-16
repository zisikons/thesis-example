function [ r_dot ] = performance_dot( t, l, r0, r_inf )
%PERFORMANCE_DOT Derivative of a performance function to be used in a controller's design.
%
% This function is used in the design of control inputs u(t) or transformed
% signals when applying the PPC methodology. When used in a simulated controller, 
% it's best to use a function pointer. For example:
% r_dot = @(t) performance_dot(t,rise_time,r0,ss_error);
% and then use r_dot(t) for other signal definitions.
%
% INPUTS
%  t       : time
%  l       : the exponential decay rate of perforamce function r(t)
%  r0      : performance function value for t = 0.
%  r_inf   : The steady state value of the performance function (can't be
%  0!)
%
% OUTPUT: 
%  r_dot       : the value of the derivative of the performance function in time t.

    r_dot = -l*(r0 - r_inf)*exp(-l*t);

end

