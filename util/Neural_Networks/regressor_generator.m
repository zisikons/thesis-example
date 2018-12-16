function [ f ] = regressor_generator( centers, sigmas, bias )
%REGRESSOR_GENERATOR Creates a function handle according to a regressor
%vector description.
%   ARGUMENTS:
%       - centers = a list of COLUMN vectors containing the centers
%       - simgas  = column vector or scalar.
%
    % Handle case where bias is requested
    if nargin == 2
      bias = false;
    end

    % Read Vector sizes
    centersDim = size(centers,1);
    centersN   = size(centers,2);
    sigmasN   = size(sigmas,1);
    
    % Case where only one sigma is given
    if  sigmasN~= 1
        error('Current implementation works only with one sigma')
        return
    %elseif centersDim ~= sigmasN
        %error('Sigmas and centers dimension mismatch!')
        %return
    end

    % Return Template
    f = @(x) regressor_template(x,centers,sigmas,bias);

end
function y = regressor_template(x,centers,sigmas,bias)

    % Vectorized norm calculation
    diffs = bsxfun(@minus,centers,x);
    total_norm = diffs.*diffs;
    total_norm = sqrt(sum(total_norm,1))';
    
    % Exponent and gaussian form
    y = exp(-(total_norm/sigmas).^2);
  
    if bias == true
      y = [1;y];
    end
end
