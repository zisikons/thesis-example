function y = S(x,M,m,c)
%S Transformation function for mapping the transformed error eps(t) from (-inf,inf) to (m,M).
%
% This function is usually not needed for simulation purposes.
%
% INPUTS
%  x       : the scalar input
%  M       : the upper bound of eps(t) = e(t)/r(t)
%  m       : the lower bound of eps(t) = e(t)/r(t)
%  c       : Parameterization coefficient

% OUTPUT: 
%  y       : the values of the parameterized function S for the x values.
    y = ( M*exp(c*x) + m*exp(-c*x) )./( exp(c*x) + exp(-c*x) );
end

