function  y  = nussbaum( x )
%NUSSBAUM Nussbaum function for cases when sign of g(x) is unknown.
%
%   Not fully parametrized yet?? Check!
%
    y = exp(0.01*x.^2).*cos(x*pi/4);
end

