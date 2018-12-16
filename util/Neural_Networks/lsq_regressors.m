function [w_star] = lsq_regressors(f, N, reg_handle, reg_size,lim)
% LSQ_REGRESSORS Method of Least Squares for regressors implementation
%
%
% ARGUMENTS:
%   f = function handle
%   N = Test samples
%   reg_handle = regressors vector function handle
%   reg_size   = size of the regressor vector
%   limits = [lim_state1_low lim_state1_upper;
%             lim_state2_low lim_state2_upper;
%                           ...              ]
%
%   regressors = regressor size and regressor handle

    % State vector dimension
    dim = size(lim,1);

    % Generate Random Dataset between given limits
    dataset = bsxfun(@times,rand(dim,N),lim(:,2)-lim(:,1));
    dataset = bsxfun(@plus,dataset,lim(:,1));
    Phi = zeros(N,reg_size);
    y   = zeros(N,1);
    
    % Calculate regressor matrix and output vector
    for i = 1:N
        Phi(i,:) = reg_handle(dataset(:,i))';
        y(i) = f(dataset(:,i));
    end
    
    % Canonical equation solution
    w_star = (Phi'*Phi)\Phi'*y;

    %% Evaluate Fit (optional)
    % Evaluation dataset generation
    N_eval = 5000;
    evaluation_dataset = bsxfun(@times,rand(dim,N_eval),lim(:,2)-lim(:,1));
    evaluation_dataset = bsxfun(@plus,evaluation_dataset,lim(:,1));
    
    % Calculation of function and function approximation over dataset
    y_lsq = zeros(N_eval,1);
    y_real = zeros(N_eval,1);
    for i = 1:N_eval
        y_lsq(i)  = w_star'*reg_handle(evaluation_dataset(:,i));
        y_real(i) = f(evaluation_dataset(:,i));
    end
    
    % Calculate NRMSE
    nrmse_eval = sqrt(immse(y_real,y_lsq))/abs(mean(y_real));
    disp(['LSQ Approximation NRMSE = ',num2str(nrmse_eval)]);
    
end