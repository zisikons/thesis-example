function [ total_index ] = index_calculator( N,regressors )
%INDEX_CALCULATOR Calcluate index matrix for the reduced state simulation
%     total_index = [1:(2*N),...
%                    2*N + regressors.index_f,2*N + regressors.index_f+ regressors.size_f,...
%                    2*N + 2*regressors.size_f + index_g,2*N + 2*regressors.size_f + regressor.size_g + index_g,...
%                    2*N + 2*regressors.size_f + 2*regressor.size_g + index_g,2*N + 2*regressors.size_f + 3*regressor.size_g + index_g];

    a = 1:(2*N);
    b = 2*N + regressors.index_f;
    c = 2*N + regressors.index_f+ regressors.size_f;
    d = 2*N + 2*regressors.size_f + regressors.index_g;
    e = 2*N + 2*regressors.size_f + regressors.size_g + regressors.index_g;
    f = 2*N + 2*regressors.size_f + 2*regressors.size_g + regressors.index_g;
    g = 2*N + 2*regressors.size_f + 3*regressors.size_g + regressors.index_g;

    total_index = [a,b,c,d,e,f,g];
end

