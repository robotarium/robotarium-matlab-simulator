%% create_si_to_uni_dynamics_with_backwards_motion
% Returns a mapping $\left( f: \mathbf{R}^{2 \times N} \times \mathbf{R}^{3
% \times N} \to \mathbf{R}^{2 \times N} \right)$
% from single-integrator to unicycle dynamics.
%
% This implementation of the mapping allows for robots to drive backwards
% if that direction of linear velocity requires less rotation.
%% Parameter Description 
% * LinearVelocityGain - affects the linear velocity for the unicycle
% * AngularVelocityLimit - affects the upper (lower) bound for the
% unicycle's angular velocity
%% Example Usage 
%   % si === single integrator
%   si_to_uni_dynamics = create_si_to_uni_dynamics_with_backwards_motion()
%   dx_si = si_algorithm(si_states) 
%   dx_uni = si_to_uni_dynamics(dx_si, states)
%% Implementation
function [si_to_uni_dyn] = create_si_to_uni_dynamics_with_backwards_motion(varargin)

    parser = inputParser;
    addOptional(parser, 'LinearVelocityGain', 1);
    addOptional(parser, 'AngularVelocityLimit', pi);
    parse(parser, varargin{:});
    
    lvg = parser.Results.LinearVelocityGain;
    avl = parser.Results.AngularVelocityLimit;
    wrap = @(x) atan2(sin(x), cos(x));
    
    si_to_uni_dyn = @si_to_uni;

    
    function dxu = si_to_uni(dxi, states)
                
        [M, N] = size(dxi); 
        [M_states, N_states] = size(states);
        
        assert(M==2, 'Column size of given SI velocities (%i) must be 2', M);
        assert(M_states==3, 'Column size of given poses (%i) must be 3', M_states);        
        assert(N==N_states, 'Row sizes of SI velocities (%i) and poses (%i) must be the same', N, N_states);
        
        dxu = zeros(2, N);
        for i = 1:N
            angle = wrap(atan2(dxi(2, i), dxi(1, i)) - states(3, i));
            if(angle > -pi/2 && angle < pi/2)
                s = 1;
            else
                s = -1;
            end
            if(s < 0)               
                states(3, i) = wrap(states(3, i) + pi);
            end
            dxu(1, i) = lvg*[cos(states(3, i)) sin(states(3, i))] * dxi(:, i);            
            %Normalizing the output of atan2 to between -kw and kw
            dxu(2, i) = avl*atan2([-sin(states(3, i)) cos(states(3, i))]*dxi(:, i), ...
                dxu(1, i))/(pi/2);
            dxu(1, i) = s*dxu(1, i);
        end
    end
end

