function [ u_ref ] = ref_control_input_poly( parameters,T )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    u_ref = [20*T.^3, 12*T.^2,6*T,2*ones(size(T,1),1),...
        zeros(size(T,1),1),zeros(size(T,1),1)]*parameters;
end

