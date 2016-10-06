%% create_si_to_uni_Mapping
% Returns a mapping from single-integrator to
% unicycle dynamics $\left( f: \mathbf{R}^{2 \times N} \times 
% \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N} \right)$ 
% and a mapping between their states $\left(f: \mathbf{R}^{3 \times N} \to
% \mathbf{R}^{2 \times N} \right)$
% Using this
% particular method, the single-integrator dynamics must be computed in the
% single-integrator domain.
%% Detailed Description 
% * ProjectionDistance - affects how far the single-integrator is projected
% in front of the unicycle system.
%% Example Usage
%   % si === single integrator
%   [si_to_uni_dynamics, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance',
%   0.05)
%   si_states = uni_to_si_states(robot_poses) 
%   dx_si = si_algorithm(si_states) 
%   dx_uni = si_to_uni_dynamics(dx_si, states)
%% Implementation
function [si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping(varargin)

    parser = inputParser;
    addOptional(parser, 'ProjectionDistance', 0.05);
    parse(parser, varargin{:});
    
    projection_distance = parser.Results.ProjectionDistance;
    
    si_to_uni_dyn = @si_to_uni;
    uni_to_si_states = @uni_to_si_states_;
    
    T = [1 0; 0 1/projection_distance];
    % First mapping from SI -> unicycle.  Keeps the projected SI system at
    % a fixed distance from the unicycle model
    function dxu = si_to_uni(dxi, states)
        N = size(dxi, 2); 
        dxu = zeros(2, N);        
        for i = 1:N
            dxu(:, i) = T * [cos(states(3, i)) sin(states(3, i)); -sin(states(3, i)) cos(states(3,i))] * dxi(:, i);
        end 
    end

    % Projects the single-integrator system a distance in front of the
    % unicycle system
    function xi = uni_to_si_states_(states)              
       xi = states(1:2, :) + projection_distance*[cos(states(3, :)) ; sin(states(3, :))];
    end
end

