function [ automatic_parking_controller ] = create_waypoint_controller(varargin)
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
%       PositionEpsilon, optional: Epsilon
%       RotationError, optional: Error tolerance for rotation
%       VelocityMagnitudeLimit, optional: Limit for velocity while driving
%       to position

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.8);
    addOptional(p, 'AngularVelocityLimit', pi);
    addOptional(p, 'PositionError', 0.03);
    addOptional(p, 'PositionEpsilon', 0.01)
    addOptional(p, 'RotationError', 0.05);
    addOptional(p, 'VelocityMagnitudeLimit', 0.15)
    parse(p, varargin{:});
    
    lin_vel_gain = p.Results.LinearVelocityGain; 
    ang_vel_limit = p.Results.AngularVelocityLimit;
    vel_mag_limit = p.Results.VelocityMagnitudeLimit;
    pos_err = p.Results.PositionError;
    pos_eps = p.Results.PositionEpsilon;
    rot_err = p.Results.RotationError;
    approach_state = [];
    
    position_controller = create_si_to_uni_dynamics('LinearVelocityGain', lin_vel_gain, ...
    'AngularVelocityLimit', ang_vel_limit);

    automatic_parking_controller = @automatic_parking_controller_;

    function [dxu, output_approach_state] = automatic_parking_controller_(states, poses, input_approach_state)
        
        N = size(states, 2);
        dxu = zeros(2, N);
        if nargin > 2
           approach_state = input_approach_state;
        elseif isempty(approach_state)
            approach_state = ones(1,N);
        end
        
        for i = 1:N
            
            wrapped = poses(3, i) - states(3, i);
            wrapped = atan2(sin(wrapped), cos(wrapped));
            
            dxi = poses(1:2, i) - states(1:2, i);
           
            % Normalize 
            norm_ = norm(dxi);
                        
            if(norm_ > pos_err - pos_eps && approach_state(i))
                if(norm_ > vel_mag_limit)
                    dxi = vel_mag_limit*dxi/norm_; 
                end
                dxu(:, i) = position_controller(dxi, states(:, i));
                
            elseif(abs(wrapped) > rot_err)
                approach_state(i) = 0;
                if(norm(dxi) > pos_err)
                    approach_state(i) = 1;
                end
                dxu(1, i) = 0;
                dxu(2, i) = 2*wrapped;
            else
                dxu(:, i) = zeros(2, 1);
            end
        end 
        
        if nargout > 1
                output_approach_state = approach_state;
        end
    end    
end


