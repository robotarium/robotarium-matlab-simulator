function [ dx ] = uni2int( dxu, x, lambda )
%DIFFEOMORPHISM Translates from single integrator to unicycle dynamics
%   dxi - single integrator control input 
%   x - unicycle states 3 x N
    
    % Initialize parameters
    [~, N] = size(dxu); 
    dx = zeros(2, N); 
    
    for i = 1:N
        dx(:, i) = [cos(x(3, i)) -lambda*sin(x(3, i)); sin(x(3, i)) lambda*cos(x(3,i))] * dxu(:, i);
    end
end

