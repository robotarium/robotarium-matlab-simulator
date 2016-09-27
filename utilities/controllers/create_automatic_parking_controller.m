function [ automatic_parking_controller ] = create_automatic_parking_controller(varargin)
%CREATE_AUTOMATIC_PARKING_CONTROLLER Returns a controller which
%automatically parks agents at a desired pose
%   parking_controller =
%   CREATE_AUTOMATIC_PARKING_CONTROLLER('ApproachAngleGain', 1,
%   'DesiredAngleGain', 1, 'RotationErrorGain', 1)
% This function returns a controller that allows for agents to be parked at
% a desired position and orientation.  When the agents are within the error
% bounds, this function will automatically stop their movement.

    persistent p
    p = inputParser;
    addOptional(p, 'ApproachAngleGain', 1);
    addOptional(p, 'DesiredAngleGain', 1); 
    addOptional(p, 'RotationErrorGain', 1);
    addOptional(p, 'PositionError', 0.01); 
    addOptional(p, 'RotationError', 0.05);
    parse(p, varargin{:});
    
    gamma = p.Results.ApproachAngleGain; 
    k = p.Results.DesiredAngleGain; 
    h = p.Results.RotationErrorGain;    
    
    init_checker = create_is_initialized('PositionError', p.Results.PositionError, ... 
    'RotationError', p.Results.RotationError);
    parking_controller = create_parking_controller('ApproachAngleGain', gamma, 'DesiredAngleGain', k, ... 
    'RotationErrorGain', h);

    automatic_parking_controller = @automatic_parking_controller_;

    function dxu = automatic_parking_controller_(states, poses)
        dxu = parking_controller(states, poses);
        [~, idxs] = init_checker(states, poses);
        
        dxu(:, idxs) = zeros(2, size(idxs, 2));
    end    
end

