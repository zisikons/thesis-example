function y = invS(x,M,m,c)
%INVS Transformation function for mapping the generalized error eps(t) = e(t)/r(t) from (m,M) to (-inf,inf).
% 
% This function is used in the design of control input u(t) when applying
% the PPC methodology.
%
% INPUTS
%  x       : the scalar input
%  M       : the upper bound of eps(t) = e(t)/r(t)
%  m       : the lower bound of eps(t) = e(t)/r(t)
%  c       : Parameterization coefficient for scaling the transformation
%            picking c = 1/2 results in ln(1+x/1-x)
%
% OUTPUT: 
%  y       : the value of the parameterized function invS in time t.
    y = (2*c)\log((-m + x)./(M - x));
end