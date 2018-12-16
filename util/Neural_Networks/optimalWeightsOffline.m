function [ Wf_star, Yf, Y_approx ] = optimalWeightsOffline( regV, f, xd )
%optimalWeightsOffline Performs Offline Fit for given Regressor Vector
%and unknown function f over the desired trajectory xd. WORKS ONLY FOR 2nd
%order systems!!!
%   
%   Arguments:
%   regV = regressor function handle (given x, returns vector Zf)
%      f = function to be approximated function handle
%     xd = matrix for points over the desired trajectory.
%
%   Outputs:
%   Wf_Star = The optimal weights vector, result of the fit (row vector)
%        Yf = The function to be approximated, evaluated over
%             the given trajectory (column vector)
%  Y_approx = The final approximation evaluated over the given
%             trajectroy (column vector)

    %% Calculate Function Output over the given trajectory
    Yf = zeros(size(xd,1),1);
    for i = 1:size(xd,1)
       Yf(i) = f(xd(i,:)');
    end
    
    %% Specify model to use in nonlinear fitting
    % Warning!: Because of the usage of arrayfun and
    %           my non-vectorized implementation of regressor_generator
    %           we cannot use this for N-Dimension systems (yet)
    modelfun = @(Wf,x) arrayfun(@(x1,x2) Wf*regV([x1;x2]) ,xd(:,1),xd(:,2));

    % Calculate Number of Gaussians [Tsobania] and create initial weights
    % vector
    N_gaussians = size(regV(xd(1,:)'),1);
    wf0 = zeros(1,N_gaussians);
    
    Wf_star = nlinfit(xd,Yf,modelfun,wf0);
    
    %% Evaluate Approximation over Trajectory
    Y_approx = zeros(size(xd,1),1);
    for i = 1:size(xd,1)
       Y_approx(i) = Wf_star*regV(xd(i,:)');
    end
end

