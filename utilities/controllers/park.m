function [ dxu ] = park( states, poses, gamma, k, h)
%PARK Summary of this function goes here
%   Detailed explanation goes here

    R = @(x) [cos(x) -sin(x) ; sin(x) cos(x)];

    N = size(states, 2);
    dxu = zeros(2, N);    
        
    for i = 1:N
        
        translate = R(-poses(3, i))*(poses(1:2, i) - states(1:2, i));                
        e = norm(translate);
        theta = atan2(translate(2), translate(1));
        alpha = theta - (states(3, i) - poses(3, i));
        alpha = atan2(sin(alpha), cos(alpha));
        
        ca = cos(alpha);
        sa = sin(alpha);
        
        dxu(1, i) = gamma * e * ca;
        dxu(2, i) = k*alpha + gamma*((ca*sa)/alpha)*(alpha + h*theta);        
    end
end



