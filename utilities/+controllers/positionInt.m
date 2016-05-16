function [ dx ] = positionInt( states, poses, lambda )
%POSITIONINT Position controller via single integrator dynamics
%   Detailed explanation goes here

    [~, N] = size(poses);
    dx = zeros(2, N);
    
    for i = 1:N
       states(1:2, i) = states(1:2, i) + lambda * [cos(states(3, i)) ; sin(states(3, i))];
       dx(:, i) = poses(1:2, i) - states(1:2, i); 
    end   
end

