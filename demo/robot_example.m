%% Setup
% Include paths
clear
close all
g = genpath('../util/');
addpath(g)

% Experimental results directory (create if non-existant)
save_results = true;
results_dir = './simulation_files/demo_experiment/';
if ~exist(results_dir,'dir')
    mkdir(results_dir)
end

% Continue previous simulation [optional]
load_sim      = false;
sim_direction = './simulation_files/demo_experiment/';
loop_start    = 0;
loop_end      = 0;

%% Setup Gaussian Networks Architecture
% 2D case: a 1x1 square in the phase plain
x = linspace(-0.5,0.5,5);
[X,Y,Z,W] = ndgrid(x,x,x,x);
centers_f = [X(:), Y(:), Z(:), W(:)]';

[X,Y] = meshgrid(x,x);
centers_g = [X(:),Y(:)]';

% Optimal Sigmas
% Selects gaussian sigma based on overlap between 2 neighbouring kernels
overlap = 0.75;
sigma = norm(centers_f(:,1) - centers_f(:,2))/(2*sqrt(-log(overlap)));  % overlap based

optimal_sigmas_f = sigma;
optimal_sigmas_g = sigma;

% Setup regressor vector handle for selected grid
% f
bias_f = true;
regressors.reg_f = regressor_generator(centers_f,optimal_sigmas_f,bias_f);
regressors.size_f = size(centers_f,2) + bias_f;

% g
bias_g = true;
regressors.reg_g = regressor_generator(centers_g,optimal_sigmas_g,bias_g);
regressors.size_g = size(centers_g,2) + bias_g;

% Iterations over desired trajectory
DT = 1;               % point 2 point transition time
threshold = 0.25;     % activation threshold for simulated gains
loops = 1;

% Calculate the active centers during each transitions for the selected network
optimal_seq = 1:size(centers_f,2);

disp('Calculating active centers for each transition...')
active_centers = [active_centers_gen_robot_poly(centers_f,centers_f,optimal_sigmas_f,bias_f,DT,threshold),...
                  active_centers_gen_robot_poly(centers_f,centers_g,optimal_sigmas_g,bias_g,DT,threshold,[1,3])];
disp('Done')

%% Prescribed Performance
% Performance Function
rise_time = 0.7;                    
ss_error  = 0.02;
r0        = 4;                              
ppc_fun.r     = @(t) performance_f(t,rise_time,r0,ss_error);
ppc_fun.r_dot = @(t) performance_dot(t,rise_time,r0,ss_error);

% Transformation
c    = 1/2;
ppc_fun.T_fun = @(t) invS(t,1,-1,c);

%% Gains
gains.l = 1;
gains.k = 30;
gains.bias_gain_f = 0.2;
gains.gaussian_gain_f = 1;
gains.s_mod_f = 0.00;

gains.bias_gain_g = 0.1;
gains.gaussian_gain_g = 0.2;

%% Plant Setup Parameters
N = 4; % order
sim_plant = @(t,x,u) robot_plant(t,x,u); % plant equations

%% Launch the simulation
if save_results == true
    save([results_dir,'experiment_parameters.mat']);
end
sim_controller
