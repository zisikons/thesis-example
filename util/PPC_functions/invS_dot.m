function [ y ] = invS_dot(x,M,m,c)
%INVS_DOT Derivative of the inverse transformation function.
% 
% This function is used in the design of control inputs u(t) or transformed
% signals when applying the PPC methodology.
%
% INPUTS
%  x       : the scalar input
%  M       : the upper bound of eps(t) = e(t)/r(t)
%  m       : the lower bound of eps(t) = e(t)/r(t)
%  c       : Parameterization coefficient 
%
% OUTPUT: 
%  y       : the values of the parameterized function invS_dot for inputs x.
    y = (2*c)\(M-m)./((x - m).*(M - x));
    
end

