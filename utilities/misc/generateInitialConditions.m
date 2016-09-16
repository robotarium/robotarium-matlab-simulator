function [ poses ] = generateInitialConditions(N)
%GENERATEINITIALCONDITIONS Summary of this function goes here
%   Detailed explanation goes here

    poses = zeros(3, N);

    persistent safetyRadius;
    
    safetyRadius = 0.2;
    width = 1.15; %1.2
    height = 0.65; %0.7

    numX = floor(width / safetyRadius);
    numY = floor(height / safetyRadius);
    values = randperm(numX * numY, N);

    for i = 1:N
       [x, y] = ind2sub([numX numY], values(i));
       x = x*safetyRadius - (width/2); 
       y = y*safetyRadius - (height/2);
       poses(1:2, i) = [x ; y];
    end
    
    poses(3, :) = (rand(1, N)*2*pi - pi);
end

