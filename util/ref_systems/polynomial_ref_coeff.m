function [ coeff ] = polynomial_ref_coeff(x0,xT,t0,T)

    % Regressor matrix definition
    y   = @(t) [t^5,t^4,t^3,t^2,t,1];
    dy  = @(t) [5*t^4,4*t^3,3*t^2,2*t,1,0];
    ddy = @(t) [20*t^3,12*t^2,6*t,2,0,0];    
    R = [y(t0);y(T);dy(t0);dy(T);ddy(t0);ddy(T)];
    
    % Solutions Vector
    Y = [x0(1);
         xT(1);
         x0(2);
         xT(2);
         x0(3);
         xT(3);];

   % solution 
   coeff = R\Y;
end

