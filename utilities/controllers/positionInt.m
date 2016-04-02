function [ dx ] = positionInt( states, poses )
%POSITIONINT Position controller via single integrator dynamics
%   Detailed explanation goes here

    [~, N] = size(poses);
    dx = zeros(2, N);
    
    for i = 1:N
       dx(:, i) = int2uni((poses(1:2, i) - states(1:2, i)), states(:, i), 0.1); 
    end   
end

