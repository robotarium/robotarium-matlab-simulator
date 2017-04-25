%% create_automatic_parking_controller
% Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$) that automatically parks agents at desired poses,
% zeroing out their velocities when the point (within a tolerance) is
% reached.
%% Detailed Description
% This function returns a controller that allows for agents to be parked at
% a desired position and orientation.  When the agents are within the error
% bounds, this function will automatically stop their movement.
%% 
% * ApproachAngleGain - affects how the unicycle approaches the desired
% position
% * DesiredAngleGain - affects how the unicycle approaches th desired angle
% * RotataionErrorGain - affects how quickly the unicycle corrects rotation
% errors
%% Example Usage 
%   parking_controller =
%   CREATE_AUTOMATIC_PARKING_CONTROLLER('ApproachAngleGain', 1,
%   'DesiredAngleGain', 1, 'RotationErrorGain', 1)
%% Implementation
function [ automatic_parking_controller ] = create_automatic_parking_controller(varargin)

    p = inputParser;
    addOptional(p, 'ApproachAngleGain', 1);
    addOptional(p, 'DesiredAngleGain', 2.7); 
    addOptional(p, 'RotationErrorGain', 1);
    addOptional(p, 'PositionError', 0.01); 
    addOptional(p, 'RotationError', 0.25);
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

