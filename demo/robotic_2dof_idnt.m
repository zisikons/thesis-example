function [ dx ] = robotic_2dof_idnt( t,x,ref_params,ppc,regressors,gains,weights,sim_plant,t_start )
%ROBOTIC_2DOF_IDNT A controller with an identification scheme used for 
%                  a 2 DOF manipulator identification experiment.
%   INPUTS:
%       t = time instance
%       x = states (see below)
%       ref_params = reference system paremeter stracture
%       ppc        = ppc function and parameter stracture
%       regressors = regressor hanlde and parameter stracture
%       gains      = controller and identifier gains (stracture)
%       weights    = wf and wg full forms (used for partial simulation) 
%       sim_plant  = simulated plant function handle
%
%
%   OUTPUTS:
%       dx = The states time derivatives (in the order given below)
%
%   STATES:
%       x(1:4) = plant states (theta1,theta_1_dot,theta_2,theta_2_dot)
%       x(5:8) = reference plant states
%       x(9:8+2*regressors.size_f_sim) = simulated Wf weights
%       x((end-4*regressors.size_g_sim):end) = simulated Wg weights
%       
%       Wf = [theta_f1;theta_f2];
%       Wg = [theta_g11;theta_g21;theta_g12;theta_g22];


    %% Return Vector initialliation
    dx = x;
    
    % Calculate reference plant updates
    u_ref = zeros(2,1);
    [dx([5,6]),u_ref(1)] = ref_plant_poly(t-t_start,x([5,6]),ref_params.theta_1);
    [dx([7,8]),u_ref(2)] = ref_plant_poly(t-t_start,x([7,8]),ref_params.theta_2);
    
    %% Calculate Function Approximations
    % Wf_hat offline and online gains
    weights.wf_hat = reshape(weights.wf_hat,regressors.size_f,2);
    weights.wf_hat(regressors.index_f,:) = reshape(x(9:(8+2*regressors.size_f_sim)),regressors.size_f_sim,2);

    % Wg_hat offline and online gains
    weights.wg_hat = reshape(weights.wg_hat,regressors.size_g,4);
    weights.wg_hat(regressors.index_g,:) = reshape(x((8+2*regressors.size_f_sim)+1:end),regressors.size_g_sim,4);

    % F(x) online approximation
    regressor_f = regressors.reg_f(x(1:4));
    f_hat = regressor_f'*weights.wf_hat;  % [f1,f2];
    
    % G(x) online approximation
    regressor_g = regressors.reg_g(x([1,3]));
    g_hat = regressor_g'*weights.wg_hat;  % [g11,g21,g12,g22]
    g_hat = reshape(g_hat,2,2);
    
    % keep only simulated gaussians
    regressor_f = regressor_f(regressors.index_f);    
    regressor_g = regressor_g(regressors.index_g);
    
    %% Errors
    e = x(1:4) - x(5:8);
    eps = e([2,4]) + gains.l*e([1,3]);
    ksi = eps/ppc.r(t);
    V   = -u_ref + gains.l*e([2,4]);
    
    %% Input 
    u_cancel = -f_hat' -g_hat*(V-ksi*ppc.r_dot(t));
    u_stabilization = -gains.k*(1./(1 - ksi.^2)).*ppc.T_fun(ksi);
    
    u = u_cancel + u_stabilization;   
    
    %% Check in region function
    notInLearningRegion = any( abs(x(1:4)) > 0.5);
    
    %% System Updates
    % Plant
    dx(1:4) = sim_plant(t,x(1:4),u);
    % (ref system is updated on top of file)
    
    % Theta f Updates
    df = ppc.r(t)\repmat(regressor_f,1,2);
    df(1,:)     = df(1,:)*gains.bias_gain_f;
    df(2:end,:) = df(2:end,:)*gains.gaussian_gain_f;
    
    df = bsxfun(@times,df,[ksi(1),ksi(2)]);
    
    % Smod F
    wf_active = reshape(x(9:(8+2*regressors.size_f_sim)),regressors.size_f_sim,2);
    df = df - gains.s_mod_f*wf_active;
    dx(9:(8+2*regressors.size_f_sim)) = df(:);
    
    % Theta g Updates
    dg = ppc.r(t)\repmat(regressor_g,1,4);
    dg(1,:)     = dg(1,:)*gains.bias_gain_g;
    dg(2:end,:) = dg(2:end,:)*gains.gaussian_gain_g;
    
    dg = bsxfun(@times,dg,[V(1),V(1),V(2),V(2)]);
    dg = bsxfun(@times,dg,[ksi(1),ksi(2),ksi(1),ksi(2)]);
    
    dx((1+end-4*regressors.size_g_sim):end) = dg(:);
    
end
function [ dx,u_ref ] = ref_plant_opitmal( t,x,parameters )

    % Optimal Input
    A = [1 parameters.T;0 1];
    t_ref = t - parameters.t_start;
    u_ref = [(6*parameters.T - 12*t_ref)/parameters.T^3,...
             (6*t_ref - 2*parameters.T)/parameters.T^2]*...
             (parameters.x_final - A*parameters.x0);
         
    dx(1) = x(2);
    dx(2) = u_ref;
    end
    function [ dx,u_ref ] = ref_plant_poly( t,x,parameters )

    % Optimal Input
    u_ref = [20*t^3, 12*t^2,6*t,2,0,0]*parameters;

         
    dx(1) = x(2);
    dx(2) = u_ref;
end


