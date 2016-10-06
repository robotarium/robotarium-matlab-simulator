%% create_is_initialized 
% Creates a function to check for initialization.  The function returns
% whether all the agents have been initialized and those that have already
% finished.
%% Detailed Description
% * PositionError - affects how close the agents are required to get to the
% desired position 
% * RotationError - affects how close the agents are required to get to the
% desired rotation
%% Example Usage 
%   initialization_checker = create_is_initialized('PositionError', 0.l,
%   'RotationError', 0.01)
%   [all_initialized, done_idxs] = initialization_checker(robot_poses,
%   desired_poses)
%% Implementation
function [ created_is_initialized ] = create_is_initialized(varargin)

    parser = inputParser;
    parser.addParameter('PositionError', 0.01);
    parser.addParameter('RotationError', 0.5);
    parse(parser, varargin{:});

    position_error = parser.Results.PositionError;
    rotation_error = parser.Results.RotationError;

    created_is_initialized = @(states, initial_conditions) is_initialized(states, initial_conditions);
        
    function [done, idxs] = is_initialized(states, initial_conditions)               
        
        [M, N] = size(states);
        [M_ic, N_ic] = size(initial_conditions);
        
        assert(M==3, 'Dimension of states (%i) must be 3', M);
        assert(M_ic==3, 'Dimension of conditions (%i) must be 3', M_ic);
        assert(N_ic==N, 'Column dimension of states (%i) and conditions (%i) must be the same', N, N_ic);
                
        wrap = @(x) atan2(sin(x), cos(x));        
        f = @(x, ic) (norm(x(1:2) - ic(1:2)) <= position_error) && (abs(wrap(x(3) - ic(3))) <= rotation_error);        
        result = arrayfun(@(x) f(states(:, x), initial_conditions(:, x)), 1:N);
        
        [done, idxs] = find(result == 1);
        done = (length(done) == N);
    end
end

