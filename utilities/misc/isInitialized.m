function [ done, idxs ] = isInitialized(states, initialConditions, varargin)
%ISINITIALIZED Summary of this function goes here
%   Detailed explanation goes here

    parser = inputParser;
    parser.addParameter('PositionError', 0.01);
    parser.addParameter('RotationError', 0.5);    
    parse(parser, varargin{:});
    
    position_error = parser.Results.PositionError;
    rotation_error = parser.Results.RotationError;

    [~, N] = size(states);    
    
    wrap = @(x) atan2(sin(x), cos(x));
    
    f = @(x, ic) (norm(x(1:2) - ic(1:2)) < position_error) && (abs(wrap(x(3) - ic(3))) < rotation_error);
    
    result = arrayfun(@(x) f(states(:, x), initialConditions(:, x)), 1:N);
    
    [done, idxs] = find(result == 1);    
    done = (length(done) == N);         
end

