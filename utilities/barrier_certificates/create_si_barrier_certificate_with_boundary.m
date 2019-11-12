function [ si_barrier_certificate ] = create_si_barrier_certificate_with_boundary(varargin)
% CREATE_SI_BARRIER_CERTIFICATE Creates a single-integrator barrier
% certificate function to avoid collisions and remain inside a bounded rectangle.
%
%   Args:
%       BarrierGain, optional: Positive double 
%       SafetyRadius, optional: Positive double
%       MagnitudeLimit, optional: Positive double
%       BoundaryPoints, optional: Array of rectangle limits [xmin, xmax, ymin, ymax]
%   
%   Returns:
%       A barrier certificate function (2xN, 2xN) -> 2xN representing the
%       barrier certificate
%
%   CREATE_SI_BARRIER_CERTIFICATE('SafetyRadius', 0.2) creates a function
%   from (2xN, 2xN) -> 2xN to keep robots at least 0.2 meters apart 
%   (measured from their centers).
%
%   CREATE_SI_BARRIER_CERTIFICATE('BarrierGain', 10e4) creates a
%   barrier certificate with a particular gain.  The higher the gain, 
%   the more quickly the robots can approach each other.  
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
    parser.addParameter('BarrierGain', 100);
    parser.addParameter('SafetyRadius', 0.15);
    parser.addParameter('MagnitudeLimit', 0.2);
    parser.addParameter('BoundaryPoints', [-1.6, 1.6, -1.0, 1.0])
    parse(parser, varargin{:})
    opts = optimoptions(@quadprog,'Display','off');

    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    magnitude_limit = parser.Results.MagnitudeLimit;
    boundary_points = parser.Results.BoundaryPoints;

    si_barrier_certificate = @barrier_certificate;

    function [ dx ] = barrier_certificate(dxi, x)
        % BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
        % certificates
        % This function accepts single-integrator dynamics and wraps them in
        % barrier certificates to ensure that collisions do not occur.  Note that
        % this algorithm bounds the magnitude of the generated output to 0.1.
        %        
        %   BARRIER_CERTIFICATE(dxi, x) modifies dxi to become collision
        %   free.
        %
        %   Example:
        %       dxi is size 2xN
        %       x is size 2xN
        %       BARRIER_CERTIFICATE(dxi, x)    
        %
        %   Notes:
        %       Try not to threshold outputs of this function.  Rather, 
        %       threshold them before calling the barrier certificate.
        
        N = size(dxi, 2);
        
        if(N < 2)
           dx = dxi;
           return 
        end
        
        x = x(1:2, :);
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = nchoosek(N, 2);
        A = zeros(num_constraints, 2*N);
        b = zeros(num_constraints, 1);
        count = 1;
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(x(1:2,i)-x(1:2,j))^2-safety_radius^2;
                A(count, (2*i-1):(2*i)) = -2*(x(:,i)-x(:,j));
                A(count, (2*j-1):(2*j)) =  2*(x(:,i)-x(:,j))';
                b(count) = gamma*h^3;
                count = count + 1;
            end
        end

        for k = 1:N
            %Pos Y
            A(count, (2*k-1):(2*k)) = [0,1];
            b(count) = 0.4*gamma*(boundary_points(4)-safety_radius/2 - x(2,k))^3;
            count = count + 1;

            %Neg Y
            A(count, (2*k-1):(2*k)) = [0,-1];
            b(count) = 0.4*gamma*(-boundary_points(3)-safety_radius/2 + x(2,k))^3;
            count = count + 1;

            %Pos X
            A(count, (2*k-1):(2*k)) = [1,0];
            b(count) = 0.4*gamma*(boundary_points(2)-safety_radius/2 - x(1,k))^3;
            count = count + 1;

            %Neg X
            A(count, (2*k-1):(2*k)) = [-1,0];
            b(count) = 0.4*gamma*(-boundary_points(1)-safety_radius/2 + x(1,k))^3;
            count = count + 1;
        end

        % Threshold control inputs before QP
        norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
        threshold = magnitude_limit;
        to_thresh = norms > threshold;
        dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
        H = 2*eye(2*N);
        f = -2*vhat;
        
        vnew = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opts);
        
        %Set robot velocities to new velocities
        dx = reshape(vnew, 2, N);
    end
end

