%% generate_initial_conditions: $\mathbf{Z}^{+} \to \mathbf{R}^{3 \times N}$
% Returns a set of random poses distributed in the Robotarium workspace
%% Example Usage 
%   initial_conditions = generate_initial_conditions(4);
%% Implementation
function [ poses ] = generate_initial_conditions(N)

    poses = zeros(3, N);
    
    safetyRadius = 0.2;
    width = 1.15;
    height = 0.55;

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

