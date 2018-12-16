function [ nrmse_idnt ] = evaluate_fit( f,f_approx,f_idnt,N,lim,plot_flag )
%EVALUATE_FIT Compare offline fit with identifier final results
%
%
%   ARGUMENTS:
%       f         = real function pointer
%       f_approx  = offline approximation function pointer 
%       f_idnt    = identifier fit function pointer
%       N         = generated dataset size
%       lim       = matrix, the domain of apprximation
%       plot_flag = boolean, if true constructs a 2D plot
%
%       for example:
%        lim = [lower,upper;  % state 1
%               lower,upper;  % state 2
%                   ...
%               lower,upper;] % state N

    % State vector dimension
    dim = size(lim,1);
        
    % Evaluation Dataset 
    % Generate Random Dataset between given limits
    dataset = bsxfun(@times,rand(dim,N),lim(:,2)-lim(:,1));
    dataset = bsxfun(@plus,dataset,lim(:,1));
    
    
    y        = zeros(N,1);
    y_approx = zeros(N,1);
    y_idnt   = zeros(N,1);
    
    for i = 1:N
       y(i)        = f(dataset(:,i)); 
       y_approx(i) = f_approx(dataset(:,i));
       y_idnt(i)   = f_idnt(dataset(:,i));
    end
    
    % MSE, RMSE, NRMSE
    mse_approx = immse(y,y_approx);
    mse_idnt   = immse(y,y_idnt);
    
    rmse_approx = sqrt(mse_approx);
    rmse_idnt   = sqrt(mse_idnt);
    
    nsrmse_approx = rmse_approx/mean(y);
    nrmse_idnt    = rmse_idnt/mean(y);
    
    disp(['LSQ Approximation NRMSE = ',num2str(nsrmse_approx)]);
    disp(['Identifier Approximation NRMSE = ',num2str(nrmse_idnt)]);

    %%  2D - Plots
    if plot_flag == true
        % Generate grid
        [X,Y] = meshgrid(lim(1,1):0.01:lim(1,2),lim(2,1):0.01:lim(2,2));
        Z = zeros(size(X));
        Z_approx = zeros(size(X));
        Z_idnt = zeros(size(X));

        % Evaluate 3 models (real, lsq and identifier's)
        for i = 1:size(X,1)
            for j = 1:size(X,2)
                Z(i,j)        = f([X(i,j);Y(i,j)]);
                Z_approx(i,j) = f_approx([X(i,j);Y(i,j)]);
                Z_idnt(i,j)   = f_idnt([X(i,j);Y(i,j)]);
            end
        end

        % LSQ and real function
        figure()
        title('LSQ Approximation')
        hold on
        surf(X,Y,Z,'FaceAlpha',0.3)
        surf(X,Y,Z_approx)
        hold off

        % Identifier and real function
        figure()
        title('Identifier Approximation')
        hold on
        surf(X,Y,Z,'FaceAlpha',0.3)
        surf(X,Y,Z_idnt)
        hold off
    end
    
end

