%% Limits for f approximation [full state regressors]
% Theta 1:
lower = 0;
upper = pi/2;
num_1   = 3;
sigma_theta_1 = (upper-lower)/sqrt(2*num_1);
limits.f.theta_1.upper = upper;
limits.f.theta_1.lower = lower;


theta1 = linspace(lower,upper,num_1);

% Theta 2:
lower = -pi/2;
upper = pi/2;
num_2   = 6;
sigma_theta_2 = (upper-lower)/sqrt(2*num_2);
limits.f.theta_2.upper = upper;
limits.f.theta_2.lower = lower;

theta2 = linspace(lower,upper,num_2);

% Theta_dot 1: (== theta1 velocity)
lower = -1.25;
upper =  1.25;
num_3 = 5;
sigma_theta_1_dot = (upper-lower)/sqrt(2*num_3);
limits.f.theta_1_dot.upper = upper;
limits.f.theta_1_dot.lower = lower;

theta1_dot = linspace(lower,upper,num_3);

% Theta_dot 2: (== theta1 velocity)
lower = -1.25;
upper =  1.25;
num_4 = 5;
limits.f.theta_2_dot.upper = upper;
limits.f.theta_2_dot.lower = lower;

theta2_dot = linspace(lower,upper,num_4);
sigma_theta_2_dot = (upper-lower)/sqrt(2*num_4);

%% Generate Grid
[X,Y,Z,W] = ndgrid(theta1,theta1_dot,theta2,theta2_dot);
centers_f = [X(:),Y(:),Z(:),W(:)];
centers_f = centers_f';

optimal_sigmas_f = [sigma_theta_1;
                    sigma_theta_1_dot;
                    sigma_theta_2;
                    sigma_theta_2_dot;];     
                
limits_f = [limits.f.theta_1.lower limits.f.theta_1.upper;...
            limits.f.theta_1_dot.lower limits.f.theta_1_dot.upper;...
            limits.f.theta_2.lower limits.f.theta_2.upper;...
            limits.f.theta_2_dot.lower limits.f.theta_2_dot.upper];
               
%% Limits for g approximation [only first derivatives]
% Theta 1:
lower = 0;
upper = pi/2;
num   = 3;
sigma_theta_1 = (upper-lower)/sqrt(2*num);
limits.g.theta_1.upper = upper;
limits.g.theta_1.lower = lower;

theta1 = linspace(lower,upper,num);

% Theta 2:
lower = -pi/2;
upper = pi/2;
num   = 6;
sigma_theta_2 = (upper-lower)/sqrt(2*num);
limits.g.theta_2.upper = upper;
limits.g.theta_2.lower = lower;

theta2 = linspace(lower,upper,num);

% Grid and sigmas
[X,Y] = ndgrid(theta1,theta2);
centers_g = [X(:),Y(:)];
centers_g = centers_g';

optimal_sigmas_g = [sigma_theta_1;
                    sigma_theta_2;];
                
limits_g = [limits.g.theta_1.lower limits.g.theta_1.upper;...
            limits.g.theta_2.lower limits.g.theta_2.upper];
            

%% Transition and correct Seq
optimal_seq = 1:(num_1*num_2*num_3*num_4);  % kanonika num1*num2*num3*num4;
DT = 1; % 1 sec

% Calculation of active centers
threshold = 0.03;
active_f = active_centers_gen_robot(centers_f,centers_f,optimal_sigmas_f,...
                    true,DT,threshold);
                
active_g = active_centers_gen_robot(centers_f,centers_g,optimal_sigmas_g,...
                    true,DT,threshold,[1,3]);

active_centers{:,1} = active_f;
active_centers{:,2} = active_g;

% Store results
save('robot_grid.mat','optimal_seq','DT','limits_f','limits_g',...
    'centers_f','optimal_sigmas_f',...
    'centers_g','optimal_sigmas_g','active_centers');
