function [ automatic_parking_controller ] = create_automatic_parking_controller(varargin)
% CREATE_AUTOMATIC_PARKING_CONTROLLER Creates a controller that drive a
% unicycle-modeled sytsem to a particular point and stops it (within
% tolerances)
%
%   
% See also CREATE_PARKING_CONTROLLER, CREATE_IS_INITIALIZED

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

