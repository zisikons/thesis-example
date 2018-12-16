function y = S_dot(x,M,m,c)
%S_DOT Derivative of the transformation function.
% 
% This function is used in the design of control inputs u(t) 
% when applying the PPC methodology in complex schemas (for example 
% in the Phd examples). In simpler applications (for example this thesis
% identificaton scheme) this function is not used.
%
% INPUTS
%  x       : the scalar input
%  M       : the upper bound of eps(t) = e(t)/r(t)
%  m       : the lower bound of eps(t) = e(t)/r(t)
%  c       : Parameterization coefficient 
%
% OUTPUT: 
%  y       : the values of the parameterized function S_dot for inputs x.
    y = -2*c*exp(2*c*x)*(m-M)./((exp(2*c*x)+1).^2);
end