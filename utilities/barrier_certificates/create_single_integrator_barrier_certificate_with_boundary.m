function [ si_barrier_certificate ] = create_single_integrator_barrier_certificate_with_boundary(varargin)
% CREATE_SINGLE_INTEGRATOR_BARRIER_CERTIFICATE_WITH_BOUNDARY 
% Creates a single-integrator barrier certificate function to avoid collisions and remain inside a defined rectangle.
%
% Returns a single-integrator barrier certificate function ($f :
% \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2
% \times N}$).  This function takes a 2 x N single-integrator
% velocity, 2 x N or 3 x N  and state vector, and returns a single-integrator
% velocity vector that does not induce collisions in the agents.
%
%   Args:
%       BarrierGain, optional: Positive double 
%       SafetyRadius, optional: Positive double
%       MagnitudeLimit, optional: Positive double
%       BoundaryPoints, optional: Array of rectangle limits to stay within [xmin, xmax, ymin, ymax]
%   
%   Returns:
%       A barrier certificate function (2xN, 2xN) -> 2xN representing the
%       barrier certificate
%
%   create_single_integrator_barrier_certificate_with_boundary('SafetyRadius', 0.2) creates a function
%   from (2xN, 2xN) -> 2xN to keep robots at least 0.2 meters apart 
%   (measured from their centers).
%
%   create_single_integrator_barrier_certificate_with_boundary('BarrierGain', 10e4) creates a
%   barrier certificate with a particular gain.  The higher the gain, 
%   the more quickly the robots can approach each other.  
%
%   Example:
%       bc = create_single_integrator_barrier_certificate_with_boundary('SafetyRadius', 0.2, 'BarrierGain', 500, 'MagnitudeLimit', 0.15, 'BoundaryPoints', [-1.6, 1.6, -1.0, 1.0])
%   
%   Notes:
%       SafetyRadius should be a positive double
%       BarrierGain should be a positive double
%       MagnitudeLimit should be a positive double
%       BoundaryPoints should be an array of doubles
%       In practice, the value for SafetyRadius should be a little more than the
%       diameter of the robots (0.15m), MagnitudeLimit should be less than 0.2, and the BoundaryPoints array
%       should be smaller than the testbed boundaries [-1.6, 1.6, -1.0, 1.0].
        
    parser = inputParser;
    parser.addParameter('BarrierGain', 100);
    parser.addParameter('SafetyRadius', 0.17);
    parser.addParameter('MagnitudeLimit', 0.15);
    parser.addParameter('BoundaryPoints', [-1.6, 1.6, -1.0, 1.0])
    parse(parser, varargin{:})
    opts = optimoptions(@quadprog,'Display','off');

    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    magnitude_limit = parser.Results.MagnitudeLimit;
    boundary_points = parser.Results.BoundaryPoints;

    %Check given inputs
    assert(isa(gamma,'numeric'), "In the function create_single_integrator_barrier_certificate_with_boundary, the barrier function gain (BarrierGain) must be a MATLAB numeric value.")
    assert(isa(safety_radius,'numeric'), "In the function create_single_integrator_barrier_certificate_with_boundary, the safety distance that two robots cannot get closer to each other than (SafetyRadius) must be a MATLAB numeric value.")
    assert(isa(magnitude_limit,'numeric'), "In the function create_single_integrator_barrier_certificate_with_boundary, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be a MATLAB numeric value.")

    assert(gamma > 0, "In the function create_single_integrator_barrier_certificate_with_boundary, the barrier function gain (BarrierGain) must be a positive value. Recieved " + num2str(gamma) + ".")
    assert(safety_radius >= 0.15, "In the function create_single_integrator_barrier_certificate_with_boundary, the safety distance that two robots cannot get closer to each other than (SafetyRadius) must be less than the diameter of a single robot (0.15m). Recieved " + num2str(safety_radius) + ".")
    assert(magnitude_limit > 0, "In the function create_single_integrator_barrier_certificate_with_boundary, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be positive. Recieved " + num2str(magnitude_limit) + ".")
    assert(magnitude_limit <= 0.2, "In the function create_single_integrator_barrier_certificate_with_boundary, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be less than or equal to the max speed of the robot (0.2m/s). Recieved " + num2str(magnitude_limit) + ".")
    assert(length(boundary_points)==4, "In the function create_single_integrator_barrier_certificate_with_boundary, the boundary points vector (BoundaryPoints) must have a length of 4 representing the rectangle verticies.")
    assert(boundary_points(2) > boundary_points(1), "In the function create_single_integrator_barrier_certificate_with_boundary, the first two entries of the boundary points vector (BoundaryPoints) representing the x-coordinates must be ordered as lower bound then upper bound.")
    assert(boundary_points(4) > boundary_points(3), "In the function create_single_integrator_barrier_certificate_with_boundary, the second two entries of the boundary points vector (BoundaryPoints) representing the y-coordinates must be ordered as lower bound then upper bound.")
    

    si_barrier_certificate = @barrier_certificate;

    function [ dxi ] = barrier_certificate(dxi, x)
        % BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
        % certificates with an outer boundary
        % This function accepts single-integrator dynamics and wraps them in
        % barrier certificates to ensure that collisions do not occur and robots 
        % stay within the defined rectangular boundary.  Note that
        % this algorithm bounds the magnitude of the generated output to the input MagnitudeLimit.
        %        
        %   BARRIER_CERTIFICATE(dxi, x) minimally modifies dxi to become collision
        %   free and stay within a defined rectangular boundary.
        %
        %
        %   Input:
        %       dxi is the single integrator control input vector size 2xN ([dx1, dx2, dx3,...,dxN;dy1,dy2,dy3,...,dyN])
        %       x is the robot pose vector ([x-pos; y-pos; heading]) size 3xN ([x1, x2, x3,...,xN;y1,y2,y3,...,yN;h1,h2,h3,...,hN])
        %
        %   Example:
        %       dxi is size 2xN
        %       x is size 3xN
        %       BARRIER_CERTIFICATE(dxi, x)
        %
        %   Returns:
        %       A 2xN matrix of safe single integrator control inputs     
        %
        %   Notes:
        %       The velocity vector input to this function will have its magnitude thresholded
        %       in an attempt to guarentee safe control inputs generated do not exceed motor limits
        %       without constraining the optimization.

       % Check given inputs
       assert(size(dxi,1) == 2, "In the function create_single_integrator_barrier_certificate_with_boundary, the single integrator vector input (dxi) must be 2xN. Recieved size" + num2str(size(dxi,1)) + "xN.")
       assert((size(x,1) == 3 || size(x,1) == 2), "In the function create_single_integrator_barrier_certificate_with_boundary, the vector input (x) must be pose 3xN or position 2xN. Recieved size" + num2str(size(x,1)) + "xN.")
       assert(size(dxi,2) == size(x,2), "In the function create_single_integrator_barrier_certificate_with_boundary, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size " + num2str(size(x,1)) + "x" + num2str(size(x,2)) + "and single integrator velocity array (dxi) of size " + num2str(size(dxi,1)) + "x" + num2str(size(dxi,2)) + ".");  

        N = size(dxi, 2);
        
        if(N < 2)
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
            b(count) = gamma*(boundary_points(4)-safety_radius/2 - x(2,k))^3;
            count = count + 1;

            %Neg Y
            A(count, (2*k-1):(2*k)) = [0,-1];
            b(count) = gamma*(-boundary_points(3)-safety_radius/2 + x(2,k))^3;
            count = count + 1;

            %Pos X
            A(count, (2*k-1):(2*k)) = [1,0];
            b(count) = gamma*(boundary_points(2)-safety_radius/2 - x(1,k))^3;
            count = count + 1;

            %Neg X
            A(count, (2*k-1):(2*k)) = [-1,0];
            b(count) = gamma*(-boundary_points(1)-safety_radius/2 + x(1,k))^3;
            count = count + 1;
        end

       % To avoid errors, we need to normalize dxi
       norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
       to_normalize = norms > magnitude_limit;
       if(size(to_normalize, 2) > 0)
           dxi(:, to_normalize) = magnitude_limit*dxi(:, to_normalize)./norms(to_normalize);
       end
       
        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
        H = 2*eye(2*N);
        f = -2*vhat;
        
        vnew = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opts);
        
        %Set robot velocities to new velocities
        dxi = reshape(vnew, 2, N);
    end
end

