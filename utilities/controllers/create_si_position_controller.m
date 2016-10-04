%% create_si_position_controller 
% Returns a controller ($u: \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2 \times N}$) 
% for a single-integrator system.
%% Detailed Description 
% * XVelocityGain - affects the horizontal velocity of the
% single integrator
% * YVelocityGain - affects the vertical velocity of the single integrator
%% Example Usage 
%   si_position_controller = create_si_position_controller('XVelocityGain',
%   1, 'YVelocityGain', 1);
%% Implementation
function [si_position_controller] = create_si_position_controller(varargin)
    
    parser = inputParser;
    addOptional(parser, 'XVelocityGain', 1);
    addOptional(parser, 'YVelocityGain', 1);
    
    parse(parser, varargin{:});
    
    x_vel_gain = parser.Results.XVelocityGain;
    y_vel_gain = parser.Results.YVelocityGain;
    gains = diag([x_vel_gain ; y_vel_gain]);
    
    si_position_controller = @(states, poses) position_si(states, poses);
    

    function [ dx ] = position_si(states, poses)
    %POSITIONINT Position controller via single integrator dynamics
    
        % Error checking
    
        [M, N] = size(states);
        [M_poses, N_poses] = size(poses); 
        
        assert(M == 2, 'Row size of states (%i) must be 2', M); 
        assert(M_poses==2, 'Row size of SI poses (%i) must be 2', M_poses);
        assert(N==N_poses, 'Column size of states (%i) must be the same as poses (%i)', N, N_poses);
        
        dx = zeros(2, N);
        for i = 1:N   
           dx(:, i) = gains*(poses(:, i) - states(:, i));
        end   
    end
end

