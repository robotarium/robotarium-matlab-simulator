function [si_position_controller] = create_si_position_controller(varargin)
    
    parser = inputParser;
    addOptional(parser, 'XVelocityGain', 1);
    addOptional(parser, 'YVelocityGain', 1);
    
    parse(parser, varargin{:});
    
    xvg = parser.Results.XVelocityGain;
    yvg = parser.Results.YVelocityGain;
    
    si_position_controller = @(states, poses) position_si(states, poses, xvg, yvg);

    function [ dx ] = position_si(states, poses, x_vel_gain, y_vel_gain)
    %POSITIONINT Position controller via single integrator dynamics
    %   Detailed explanation goes here
    
        % Error checking
    
        [M, N] = size(states);
        [M_poses, N_poses] = size(poses); 
        
        assert(M == 3, 'Row size of states (%i) must be 3', M); 
        assert(M_poses==2, 'Row size of SI poses (%i) must be 2', M_poses);
        assert(N==N_poses, 'Column size of states (%i) must be the same as poses (%i)', N, N_poses);
        
        dx = zeros(2, N);
        gains = diag([x_vel_gain ; y_vel_gain]);
        for i = 1:N   
           dx(:, i) = gains*(poses(1:2, i) - states(1:2, i));
        end   
    end
end

