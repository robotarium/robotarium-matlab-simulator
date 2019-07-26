function [ poses ] = generate_initial_conditions(N, varargin)
% GENERATE_INITIAL_CONDITIONS generate random poses in a circle
% The default parameter values are correctly sized for the Robotarium's
% physical testbed.
%
%   GENERATE_INITIAL_CONDITIONS(5) generates 3 x 5 matrix of
%   random poses
%
%   GENERATE_INITIAL_CONDITIONS(5, 'Spacing', 0.2, 'Width',
%   3.2, 'Height', 2) generates 3 x 5 matrix of random poses with
%   spacing 0.2 m in a rectangle of 3.2 m width and 2 m height.
%
%   Example:
%      poses = generate_initial_conditions(5);
%   
%   Notes:
%       N should be a positive integer.
    
    poses = zeros(3, N);
    
    parser = inputParser;
    parser.addParameter('Spacing', 0.3);
    parser.addParameter('Width', 3.0);
    parser.addParameter('Height', 1.8);
    parse(parser, varargin{:});
    
    spacing = parser.Results.Spacing;
    width = parser.Results.Width;
    height = parser.Results.Height;

    numX = floor(width / spacing);
    numY = floor(height / spacing);
    values = randperm(numX * numY, N);

    for i = 1:N
       [x, y] = ind2sub([numX numY], values(i));
       x = x*spacing - (width/2); 
       y = y*spacing - (height/2);
       poses(1:2, i) = [x ; y];
    end
    
    poses(3, :) = (rand(1, N)*2*pi - pi);
end

