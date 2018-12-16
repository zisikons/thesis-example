function [ dx ] = ref_system_poly( t,x,parameters )

    % Laws
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = [20*t^3, 12*t^2,6*t,2,0,0]*parameters;

end

