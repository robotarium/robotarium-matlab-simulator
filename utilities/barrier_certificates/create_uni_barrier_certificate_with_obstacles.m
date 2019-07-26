function [ uni_barrier_certificate ] = create_uni_barrier_certificate_with_obstacles(varargin)
% CREATE_UNI_BARRIER_CERTIFICATE_WITH_OBSTACLES Creates a barrier certificate for a
% unicycle-modeled systems
% Works by projecting a virtual single-integrator system ahead of the
% unicycle and applying a suitably large barrier certificate to the virtual
% system.
%
%   Args:
%       BarrierGain, optional: A gain for how quickly the system can
%       approach obstacles
%       SafetyRadius, optional: How far points should remain apart.
%       Nominal value is 1.5*robot_diameter
%       ProjectionDistance: How far ahead to place the virtual
%       single-integrator system
%       VelocityMagnitudeLimit: Limit for the magnitude of the virtual
%       single integrator
%
%   Returns:
%       A barrier function (2xN, 3xN, 2xN) -> 2xN that generates safe
%       control inputs for unicycle-modeled systems.
%
%   CREATE_UNI_BARRIER_CERTIFICATE_WITH_OBSTACLES('BarrierGain', 8e3)
%
%   CREATE_UNI_BARRIER_CERTIFICATE_WITH_OBSTACLES('SafetyRadius', 0.15)
%
%   CREATE_UNI_BARRIER_CERTIFICATE_WITH_OBSTACLES('ProjectionDistance',
%   0.05)
%
%   CREATE_UNI_BARRIER_CERTIFICATE_WITH_OBSTACLES('VelocityMagnitudeLimit',
%   0.4)

    parser = inputParser;
    addOptional(parser, 'BarrierGain', 100);
    addOptional(parser, 'SafetyRadius', 0.15);
    addOptional(parser, 'ProjectionDistance', 0.05)
    addOptional(parser, 'VelocityMagnitudeLimit', 0.2);
    parse(parser, varargin{:})
    
    opts = optimoptions(@quadprog,'Display','off');       
    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    projection_distance = parser.Results.ProjectionDistance;
    velocity_magnitude_limit = parser.Results.VelocityMagnitudeLimit;
    
    [si_uni_dyn, uni_si_states] = create_si_to_uni_mapping('ProjectionDistance', projection_distance);
    uni_si_dyn = create_uni_to_si_mapping('ProjectionDistance', projection_distance);
    
    uni_barrier_certificate = @barrier_unicycle;

    function [ dxu ] = barrier_unicycle(dxu, x, obstacles)      
        N = size(dxu, 2);
        
        %Shift to single integrator domain
        xi = uni_si_states(x);
        dxi = uni_si_dyn(dxu, x);
               
        % Normalize velocities
        norms = arrayfun(@(idx) norm(dxi(:, idx)), 1:N);
        to_normalize = norms > velocity_magnitude_limit;
        
        if(any(to_normalize == 1))
            dxi(:, to_normalize) = velocity_magnitude_limit*dxi(:, to_normalize)./norms(to_normalize);
        end
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        if(N > 2)
           temp = nchoosek(N, 2);
        else
            temp = N - 1;
        end
        num_constraints = temp + size(obstacles, 2);
        A = zeros(num_constraints, 2*N);
        b = zeros(num_constraints, 1);
        count = 1;
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(xi(:,i)-xi(:,j))^2-(safety_radius + 2*projection_distance)^2;
                A(count, (2*i-1):(2*i)) = 2*(xi(:,i)-xi(:,j))';
                A(count, (2*j-1):(2*j)) = -2*(xi(:,i)-xi(:,j))';
                b(count) = -gamma*h^3;
                count = count + 1;
            end
        end
        
        % Do obstacles
        for i = 1:N
            for j = 1:size(obstacles, 2)
                h = norm(xi(:,i)-obstacles(:,j))^2-(safety_radius + projection_distance)^2;
                A(count, (2*i-1):(2*i)) = 2*(xi(:,i)-obstacles(:,j))';
                b(count) = -gamma*h^3;
                count = count + 1;
            end
        end
        
        A = -A;
        b = -b;
        
        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
        H = 2*eye(2*N);
        f = -2*vhat;
        
        vnew = quadprog(sparse(H), double(f), A, b, [], [], [], [], [], opts);
        
        %Set robot velocities to new velocities
        dxu = si_uni_dyn(reshape(vnew, 2, N), x);

    end
end

