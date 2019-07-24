function [ uni_barrier_certificate ] = create_uni_barrier_certificate(varargin)
% CREATE_SI_BARRIER_CERTIFICATE Creates a unicycle barrier
% certificate function to avoid collisions.
%
%   Args:
%       BarrierGain, optional: How quickly robots can approach eachother
%       SafetyRadius, optional: How far apart centers of robots should
%       remain
%       ProjectionDistance, optional: How far ahead to project a virtual
%       single integrator
%       VelocityMagnitudeLimit, optional: The maximum velocity for the
%       virtual single integrator
%   
%   Returns:
%       A barrier certificate function (2xN, 3xN) -> 2xN representing the
%       barrier certificate
%
%   CREATE_UNI_BARRIER_CERTIFICATE('BarrierGain', bg)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr, 'BarrierGain', bg)
%
%   Example:
%       bc = create_si_barrier_certificate('SafetyRadius', 0.2)
%   
%   Notes:
%       SafetyRadius should be a positive double
%       BarrierGain should be a positive double
%       In practice, the value for SafetyRadius should be a little more than double the
%       size of the robots.

    parser = inputParser;
    addOptional(parser, 'BarrierGain', 100);
    addOptional(parser, 'SafetyRadius', 0.15);
    addOptional(parser, 'ProjectionDistance', 0.05);
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

    function [ dxu ] = barrier_unicycle(dxu, x)   
        % BARRIER_UNICYCLE The parameterized barrier function
        %
        %   Args:
        %       dxu: 2xN vector of unicycle control inputs
        %       x: 3xN vector of unicycle states
        %
        %   Returns:
        %       A 2xN matrix of safe unicycle control inputs
        %
        %   BARRIER_UNICYCLE(dxu, x)
        
        N = size(dxu, 2);
        
        if(N < 2)
           return 
        end
        
        %Shift to single integrator domain
        xi = uni_si_states(x);
        dxi = uni_si_dyn(dxu, x);
               
        % Normalize velocities
        norms = arrayfun(@(idx) norm(dxi(:, idx)), 1:N);
        to_normalize = norms > velocity_magnitude_limit;
        if(size(to_normalize, 2) > 0)
            dxi(:, to_normalize) = velocity_magnitude_limit*dxi(:, to_normalize)./norms(to_normalize);
        end
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = nchoosek(N, 2);
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

