function [active_centers] = active_centers_gen_2d(trajectory_centers,regressor_centers,sigmas,bias,DT,threshold)
%ACTIVE_CENTERS_GEN_2D
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
    
    % Addpath to regrssor generator
    addpath ../
    
    % Calculate system size
    centersN = size(trajectory_centers,2);
    regressorN = size(regressor_centers,2)+bias;
    
    optimal_seq = 1:centersN;
    seq = [optimal_seq,1];      % helper variable
    active_centers = cell(centersN,1);
    
    % Define Regressor function, given the centers
    reg_fun = regressor_generator(regressor_centers,sigmas,bias);
    
    % Simulation Centers
    for i = optimal_seq
        
        % Setup Reference System
        ref.T = DT;
        ref.t_start = 0;
        ref.x0 = trajectory_centers(:,seq(i));
        ref.x_final = trajectory_centers(:,seq(i+1));
        
        % ODE Simulation
        fun = @(t,x) ref_plant_2nd_order(t,x,ref);
        T_sim = [0 DT];
        x0 = ref.x0;
        [~,X] = ode15s(fun,T_sim,x0);
        
        % Regressor Evaluation
        active = [];
        for j =1:size(X,1)
            % Find index of activated centers
            logical_idx = reg_fun(X(j,:)')>=threshold;
            temp = logical_idx.*(1:regressorN)';
            temp = temp(temp~=0); 
            active = unique([active;temp]);
        end 
        active_centers{i} = active;
    end
   
end
function [ dx ] = ref_plant_2nd_order( t,x,parameters )
    % Optimal Input
    A = [1 parameters.T;0 1];
    t_ref = t - parameters.t_start;
    u_ref = [(6*parameters.T - 12*t_ref)/parameters.T^3,...
             (6*t_ref - 2*parameters.T)/parameters.T^2]*...
             (parameters.x_final - A*parameters.x0);

    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = u_ref;
end
