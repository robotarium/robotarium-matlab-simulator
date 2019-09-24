function [ automatic_parking_controller ] = create_automatic_parking_controller2(varargin)
% CREATE_AUTOMATIC_PARKING_CONTROLLER Creates a controller that drive a
% unicycle-modeled sytsem to a particular point and stops it (within
% tolerances)
% Works by driving the unicycle to within PositionError of the point then
% rotating it to within RotationError of the desired rotation
%
%   Args:
%       LinearVelocityGain, optional: see also
%       AngularVelocityLimit, optional: see also
%       PositionError, optional: Error tolerance for position
%       RotationError, optional: Error tolerance for rotation
%       VelocityMagnitudeLimit, optional: Limit for velocity while driving
%       to position
%
% See also CREATE_SI_TO_UNI_MAPPING3

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.5);
    addOptional(p, 'AngularVelocityLimit', pi/2);
    addOptional(p, 'PositionError', 0.03); 
    addOptional(p, 'RotationError', 0.25);
    addOptional(p, 'VelocityMagnitudeLimit', 0.2)
    parse(p, varargin{:});
    
    lin_vel_gain = p.Results.LinearVelocityGain; 
    ang_vel_gain = p.Results.AngularVelocityLimit;
    vel_mag_limit = p.Results.VelocityMagnitudeLimit;
    pos_err = p.Results.PositionError;
    rot_err = p.Results.RotationError;
    
    position_controller = create_si_to_uni_dynamics('LinearVelocityGain', lin_vel_gain, ...
    'AngularVelocityLimit', ang_vel_gain);

    automatic_parking_controller = @automatic_parking_controller_;

    function dxu = automatic_parking_controller_(states, poses)
        
        N = size(states, 2);
        dxu = zeros(2, N);
        
        for i = 1:N
            
            wrapped = poses(3, i) - states(3, i);
            wrapped = atan2(sin(wrapped), cos(wrapped));
            
            dxi = poses(1:2, i) - states(1:2, i);
            
            % Normalize 
            norm_ = norm(dxi);
            if(norm_ > vel_mag_limit)
               dxi = vel_mag_limit*dxi/norm_; 
            end
            
            if(norm(dxi) > pos_err)
                dxu(:, i) = position_controller(dxi, states(:, i));
            elseif(abs(wrapped) > rot_err) 
                dxu(1, i) = 0;
                dxu(2, i) = wrapped;
            else
                dxu(:, i) = zeros(2, 1);
            end            
        end       
    end    
end


