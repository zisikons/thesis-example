% Loads and plots a set of RBF-NN weights from a
% simulation directory
%
% Specify:
%    i) results_dir + exp_name to select an experiment
%   ii) loops rannge to plot (loops)
%  iii) reg_indexes_f || g to specify which weights to plot
%
% Experiment directory
results_dir = './simulation_files';
exp_name = '/demo_experiment/';
results_dir = [results_dir, exp_name];
load([results_dir,'experiment_parameters.mat']);


% Loops and regressors to be plotted
loops = 1:50;
reg_indexes_f = 0 + [1,2,3];
reg_indexes_g = 1:10;
idx_fg   = 2*N + 0*regressors.size_f + reg_indexes_f;   % wfg
idx_ginv = 2*N + 2*regressors.size_f ...
           + 2*regressors.size_g + reg_indexes_g;   % wginv

T = [];
Wfg = [];
Wginv = [];
for i = loops
   load([results_dir,'sim_loop_',num2str(i)]);
   T  = [T;T_total];
   Wfg   = [Wfg;X_total(:,idx_fg)];
   Wginv = [Wginv;X_total(:,idx_ginv)];
   clear X_total;
   clear T_total;
   clear UR_total;
   disp(['Loop ',num2str(i),'/',num2str(loops(end))]);
end

sample_rate = 1;
figure()
box on
plot(T(1:sample_rate:end),Wfg(1:sample_rate:end,:))
xlabel('$t$','Interpreter','latex')
set(gcf, 'Color', 'w');


figure()
box on
plot(T(1:sample_rate:end),Wginv(1:sample_rate:end,:))
xlabel('$t$','Interpreter','latex')
set(gcf, 'Color', 'w');
