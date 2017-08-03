%% generate_initial_conditions: $\mathbf{Z}^{+} \to \mathbf{R}^{3 \times N}$
% Returns a set of random poses distributed in the Robotarium workspace
%% Example Usage 
%   initial_conditions = generate_initial_conditions(4);
%% Implementation
function [ poses ] = generate_initial_conditions(N, varargin)

    poses = zeros(3, N);
    
    parser = inputParser;
    parser.addParameter('Spacing', 0.1);
    parser.addParameter('Width', 1.15);
    parser.addParameter('Height', 0.55);
    parse(parser, varargin{:});
    
    spacing = parser.Results.Spacing;
    rotation_error = parser.Results.Spacing;
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

