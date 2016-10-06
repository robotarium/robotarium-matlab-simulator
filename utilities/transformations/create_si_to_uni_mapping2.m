%% create_si_to_uni_mapping2 
% Returns a mapping $\left( f: \mathbf{R}^{2 \times N} \times \mathbf{R}^{3
% \times N} \to \mathbf{R}^{2 \times N} \right)$
% from single-integrator to unicycle dynamics
%% Detailed Description 
% * LinearVelocityGain - affects the linear velocity for the unicycle
% * AngularVelocityLimit - affects the upper (lower) bound for the
% unicycle's angular velocity
%% Example Usage 
%   % si === single integrator
%   si_to_uni_dynamics = create_si_to_uni_mapping2('LinearVelocityGain',
%   1, 'AngularVelocityLimit', pi)
%   dx_si = si_algorithm(si_states) 
%   dx_uni = si_to_uni_dynamics(dx_si, states)
%% Implementation
function [si_to_uni_dyn] = create_si_to_uni_mapping2(varargin)

    parser = inputParser;
    addOptional(parser, 'LinearVelocityGain', 1);
    addOptional(parser, 'AngularVelocityLimit', pi);
    parse(parser, varargin{:});
    
    lvg = parser.Results.LinearVelocityGain;
    avl = parser.Results.AngularVelocityLimit;
    
    si_to_uni_dyn = @si_to_uni;
    % A mapping from si -> uni dynamics.  THis is more of a
    % projection-based method.  Though, it's definitely similar to the
    % NIDs.
    function dxu = si_to_uni(dxi, states)
                
        [M, N] = size(dxi); 
        [M_states, N_states] = size(states);
        
        assert(M==2, 'Column size of given SI velocities (%i) must be 2', M);
        assert(M_states==3, 'Column size of given poses (%i) must be 3', M_states);        
        assert(N==N_states, 'Row sizes of SI velocities (%i) and poses (%i) must be the same', N, N_states);
        
        dxu = zeros(2, N);
        for i = 1:N
            dxu(1, i) = lvg * [cos(states(3, i)) sin(states(3, i))] * dxi(:, i);
            %Normalizing the output of atan2 to between -kw and kw
            dxu(2, i) = avl * atan2([-sin(states(3, i)) cos(states(3, i))]*dxi(:, i), ...
                                  [cos(states(3, i)) sin(states(3, i))]*dxi(:, i))/(pi/2);
        end
    end
end

