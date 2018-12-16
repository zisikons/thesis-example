function [ r ] = performance_f( t, l, r0, r_inf )
%PERFORMANCE_F Performance Function for describing a prescribed perfomance.
%
% When used in a simulated controllerz, it's best to use a function pointer. For example:
% r = @(t) performance_f(t,rise_time,r0,ss_error);
% and then use r(t) for other signal definitions.
%
% INPUTS
%  t       : time
%  l       : the exponential decay rate of perforamce function r(t)
%  r0      : performance function value for t = 0.
%  r_inf   : The steady state value of the performance function (can't be
%  0!)
%
% OUTPUT: 
%  r       : the value of the performance function in time t.

    r = (r0 - r_inf)*exp(-l*t) + r_inf;
end
