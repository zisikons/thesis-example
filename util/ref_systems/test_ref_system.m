x0.x1  = 0;
x0.x2  = 0;
x0.dx2 = 0;

xT.x1 = pi/2;
xT.x2 = 0;
xT.dx2 = 0;

t0 = 0;
T = 5;

[ coeff ] = polynomial_ref_coeff(x0,xT,t0,T);

% 
ref_plant = @(t,x) ref_system_poly( t,x,coeff );


xS = [x0.x1;x0.x2];
T_sim = [t0,T];

[T_total,X_total] = ode45(ref_plant,T_sim,xS);

plot(X_total(:,1),X_total(:,2));