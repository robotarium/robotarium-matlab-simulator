function [ dx ] = positionCLF(states, poses)
%POSITIONCLF Summary of this function goes here
%   Detailed explanation goes here

    [~, N] = size(states);
    dx = zeros(2, N);

    for i = 1:N
        dx_ = poses(1, i) - states(1, i);
        dy = poses(2, i) - states(2, i);
        dt = atan2(dy, dx_);

        dist = sqrt(dx_^2 + dy^2);

        dx(1, i) = dist*cos(dt - states(3, i));
        dx(2, i) = dist*sin(dt - states(3, i));
    end 
end

