function dx = robot_plant(t,x,u)

    % Constants
    I = [0.96, 0.81];   % inertia
    m = [3.2, 2];       % mass
    l = [0.5, 0.4];     % lengths
    g = 9.81;           % gravity constant
    
    % Inertia Matrix
    M11 = I(1)+I(2) + 0.25*m(1)*l(1)^2 +...
          m(2)*( l(1)^2 + 0.25*l(2)^2 + l(1)*l(2)*cos(x(3)));
      
    M12 = I(2) + m(2)*( 0.25*l(2)^2 + 0.5*l(1)*l(2)*cos(x(3)));
    M22 = I(2) + 0.25*m(2)*l(2)^2;
      
    M = [M11,M12;M12,M22];
    
    % Corriolis Forces
    c = 0.25*m(2)*l(1)*l(2)*sin(x(3));
    C = [-c*x(4), -c*(x(2)+x(4));
         c*x(2),0];
    
    % Gravity Vector
    G = [0.5*m(1)*g*l(1)*cos(x(1))+...
         m(2)*g*(l(1)*cos(x(1)) + 0.5*l(2)*cos(x(1)+x(3)));
         0.5*m(2)*g*l(2)*cos(x(1)+x(3))];
    
    % Maninpulator Equations 
    q_dot = M\(-C*[x(2);x(4)] - G + u);
   
    %% Final Derivatives
    dx = zeros(4,1);
    dx(1) = x(2);
    dx(2) = q_dot(1);
    dx(3) = x(4);
    dx(4) = q_dot(2);
end
