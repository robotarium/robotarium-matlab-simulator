function [ dxu ] = automaticPark(states, desiredPoses, varargin)
%ISINITIALIZED Summary of this function goes here
%   Detailed explanation goes here

    persistent parser

    parser = inputParser;
    parser.addParameter('PositionError', 0.01);
    parser.addParameter('RotationError', 0.5);   
    parser.addParameter('y', 0.7);    
    parser.addParameter('k', 5);
    parser.addParameter('h', 0.4);
    parse(parser, varargin{:});    
    position_error = parser.Results.PositionError; 
    rotation_error = parser.Results.RotationError; 
    
    
    dxu = park(states, desiredPoses, parser.Results.y,  parser.Results.k,  parser.Results.h);      
    
    [~, idxs] = isInitialized(states, desiredPoses, 'PositionError', position_error, 'RotationError', rotation_error);  
    dxu(:, idxs) = zeros(2, length(idxs));
end

