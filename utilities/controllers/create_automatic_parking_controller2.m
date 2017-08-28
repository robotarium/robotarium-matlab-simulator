%% create_automatic_parking_controller2
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
%   CREATE_AUTOMATIC_PARKING_CONTROLLER2('ApproachAngleGain', 1,
%   'DesiredAngleGain', 1, 'RotationErrorGain', 1)
%% Implementation
function [ automatic_parking_controller ] = create_automatic_parking_controller2(varargin)

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.75);
    addOptional(p, 'AngularVelocityLimit', pi/2);
    addOptional(p, 'PositionError', 0.01); 
    addOptional(p, 'RotationError', 0.25);
    addOptional(p, 'VelocityMagnitudeLimit', 0.08)
    parse(p, varargin{:});
    
    lin_vel_gain = p.Results.LinearVelocityGain; 
    ang_vel_gain = p.Results.AngularVelocityLimit;
    vel_mag_limit = p.Results.VelocityMagnitudeLimit;
    pos_err = p.Results.PositionError;
    rot_err = p.Results.RotationError;
    
    position_controller = create_si_to_uni_mapping2('LinearVelocityGain', lin_vel_gain, ...
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


