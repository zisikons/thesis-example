function [active_centers] = active_centers_gen_robot_poly(trajectory_centers,regressor_centers,sigmas,bias,DT,threshold,active_states)
%ACTIVE_CENTERS_GEN_ROBOT
%
%   Inputs:
%       - trajectory_centers : Matrix containing the target centers
%       coordinates (as columns)
%       - regressor_centers : Matrix containing the regressor centers
%       coordinates (as columns)
%       - sigmas    : Column vector containing the sigma for each regressor
%       dimension.
%       - bias      : Boolean, regressor contrains bias or not
%       - DT        : Reference System transition duration
%       - threshold : After which value, gaussian is considered "activated"
%       - 

    %% Setup
    % Addpath to regrssor generator
    addpath ../../
    
    % Calculate system size
    centersN     = size(trajectory_centers,2);
    systemDim     = size(trajectory_centers,1);
    regressorN   = size(regressor_centers,2)+bias;
    regressorDim = size(regressor_centers,1);
    
    optimal_seq = 1:centersN;
    seq = [optimal_seq,1];      % helper variable
    active_centers = cell(centersN,1);
    
    % Handle case when we have regressor of lower dimension
    if systemDim == regressorDim
        active_states = (1:systemDim)';
    elseif systemDim~=regressorDim && nargin<7
        disp('Undefined active states.')
        return
    end
    
    % Regressor Definition, given the centers and sigma(s)
    reg_fun = regressor_generator(regressor_centers,sigmas,bias);
    

    %% Calculations

    for i = optimal_seq
%         % Theta 1
%         ref.theta1.T = DT;
%         ref.theta1.t_start = 0;
%         ref.theta1.x0 = trajectory_centers([1,2],seq(i));
%         ref.theta1.x_final = trajectory_centers([1,2],seq(i+1));
% 
%         % Theta 2
%         ref.theta2.T = DT;
%         ref.theta2.t_start = 0;
%         ref.theta2.x0 = trajectory_centers([3,4],seq(i));
%         ref.theta2.x_final = trajectory_centers([3,4],seq(i+1));
        
        c0 = [trajectory_centers([1,2],seq(i));0];
        cT = [trajectory_centers([1,2],seq(i+1));0];
        ref.theta1 = polynomial_ref_coeff(c0,cT,0,DT);

        c0 = [trajectory_centers([3,4],seq(i));0];
        cT = [trajectory_centers([3,4],seq(i+1));0];
        ref.theta2 = polynomial_ref_coeff(c0,cT,0,DT);
        
        

        % Simulate
        fun = @(t,x) ref_sys(t,x,ref);
        T_sim = [0 DT];
        x0 = trajectory_centers(:,i);

        [~,X] = ode15s(fun,T_sim,x0);

        % Regressor Evaluation
        active = [];
        for j =1:size(X,1)
            % Find index of activated centers
            logical_idx = reg_fun(X(j,active_states)')>=threshold;
            temp = logical_idx.*(1:regressorN)';
            temp = temp(temp~=0); 
            active = unique([active;temp]);
        end 
        active_centers{i} = active;
    end
end
function [ dx,u_ref ] = ref_plant_poly( t,x,parameters )

% Optimal Input
u_ref = [20*t^3, 12*t^2,6*t,2,0,0]*parameters;

     
dx(1) = x(2);
dx(2) = u_ref;
end
function dx = ref_sys(t,x,ref_params)
    dx = zeros(4,1);
    dx(1:2) = ref_plant_poly(t,x([1,2]),ref_params.theta1);
    dx(3:4) = ref_plant_poly(t,x([3,4]),ref_params.theta2);
end
