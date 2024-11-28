function [ uni_barrier_certificate ] = create_unicycle_barrier_certificate(varargin)
% CREATE_SI_BARRIER_CERTIFICATE Creates a barrier
% certificate function taking in unicycle dynamics to avoid collisions.
%
% Returns a single-integrator barrier certificate function ($f :
% \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2
% \times N}$).  This function takes a 2 x N single-integrator
% velocity, 3 x N  and state vector, and returns a unicycle-integrator
% velocity vector that does not induce collisions in the agents. This is done by
% transforming the desired unicycle commands to a single integrator command through
% a near identity diffeomorphism. This is done as putting barriers on unicycle commands
% usually results in robots stopping at boundaries (as linear velocity is the only thing
% that impacts state).
%
%   Args:
%       BarrierGain, optional: How quickly robots can approach eachother
%       SafetyRadius, optional: How far apart centers of robots should
%       remain
%       ProjectionDistance, optional: How far ahead to project a virtual
%       single integrator for the near-identity diffeomorphism
%       MagnitudeLimit, optional: The maximum magnitude (velocity) for the
%       virtual single integrator vector
%   
%   Returns:
%       A barrier certificate function (2xN, 3xN) -> 2xN representing the
%       barrier certificate
%
%   CREATE_UNICYCLE_BARRIER_CERTIFICATE('SafetyRadius', 0.2) creates a function
%   from (2xN, 2xN) -> 2xN to keep robots at least 0.2 meters apart 
%   (measured from their centers).
%
%   CREATE_UNICYCLE_BARRIER_CERTIFICATE('BarrierGain', 10e4) creates a
%   barrier certificate with a particular gain.  The higher the gain, 
%   the more quickly the robots can approach each other.  
%
%   Example:
%       bc = create_unicycle_barrier_certificate('SafetyRadius', 0.2, 'BarrierGain', 500, 'MagnitudeLimit', 0.15, 'ProjectionDistance', 0.03)
%   
%   Notes:
%       In practice, the value for SafetyRadius should be a little more than the
%       diameter of the robots (0.15m), MagnitudeLimit should be less than 0.2m/s, and the ProjectionDistance should be relatively small (~0.03). Higher magnitude limits may cause output velocities to be unsafe due to motor limitations. Very small projection distances will cause very large angular velocities to be generated.

    parser = inputParser;
    addOptional(parser, 'BarrierGain', 100);%unitless
    addOptional(parser, 'SafetyRadius', 0.17);%meters
    addOptional(parser, 'ProjectionDistance', 0.03);%meters
    addOptional(parser, 'MagnitudeLimit', 0.15);%m/s
    parse(parser, varargin{:})
    
    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    projection_distance = parser.Results.ProjectionDistance;
    magnitude_limit = parser.Results.MagnitudeLimit;

    %Check given inputs
    assert(isa(gamma,'numeric'), "In the function create_unicycle_barrier_certificate, the barrier function gain (BarrierGain) must be a MATLAB numeric value.")
    assert(isa(safety_radius,'numeric'), "In the function create_unicycle_barrier_certificate, the safety distance that two robots cannot get closer to each other than (SafetyRadius) must be a MATLAB numeric value.")
    assert(isa(magnitude_limit,'numeric'), "In the function create_unicycle_barrier_certificate, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be a MATLAB numeric value.")
    assert(isa(projection_distance,'numeric'), "In the function create_unicycle_barrier_certificate, the projection distance for a virtual single integrator to create the near-identity diffeomorphism must be a MATLAB numeric value.")

    assert(gamma > 0, "In the function create_unicycle_barrier_certificate, the barrier function gain (BarrierGain) must be a positive value. Recieved " + num2str(gamma) + ".")
    assert(safety_radius >= 0.15, "In the function create_unicycle_barrier_certificate, the safety distance that two robots cannot get closer to each other than (SafetyRadius) must be less than the diameter of a single robot (0.15m). Recieved " + num2str(safety_radius) + ".")
    assert(magnitude_limit > 0, "In the function create_unicycle_barrier_certificate, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be positive. Recieved " + num2str(magnitude_limit) + ".")
    assert(magnitude_limit <= 0.2, "In the function create_unicycle_barrier_certificate, the maximum magnitude of the velocity vector for the robot to follow (MagnitudeLimit) must be less than or equal to the max speed of the robot (0.2m/s). Recieved " + num2str(magnitude_limit) + ".")
    
    
    [si_uni_dyn, uni_si_states] = create_si_to_uni_mapping('ProjectionDistance', projection_distance);
    uni_si_dyn = create_uni_to_si_mapping('ProjectionDistance', projection_distance);
    si_barrier_certificate = create_single_integrator_barrier_certificate('BarrierGain', gamma, 'SafetyRadius', safety_radius, 'MagnitudeLimit', magnitude_limit);
    
    uni_barrier_certificate = @barrier_unicycle;

    function [ dxu ] = barrier_unicycle(dxu, x)   
        % BARRIER_UNICYCLE The parameterized barrier function
        %
        % This function accepts unicycle control commands and wraps them in
        % barrier certificates to ensure that collisions do not occur.  Note that
        % this algorithm bounds the velocity magnitude of the generated output to the input MagnitudeLimit.
        %        
        %   BARRIER_CERTIFICATE(dxu, x) minimally modifies dxu to become collision
        %   free.
        %
        %
        %   Input:
        %       dxu is the single integrator control input vector size 2xN ([v1, v2, v3,...,vN;w1,w2,w3,...,wN])
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
        assert(size(dxu,1) == 2, "In the function created by create_unicycle_barrier_certificate, the unicycle control vector input (dxu) must be 2xN. Recieved size" + num2str(size(dxu,1)) + "xN.")
        assert((size(x,1) == 3), "In the function created by create_unicycle_barrier_certificate, the vector input (x) must be pose 3xN. Recieved size" + num2str(size(x,1)) + "xN.") 
        assert(size(dxu,2) == size(x,2), "In the function create_unicycle_barrier_certificate, the number of robot states (x) must be equal to the number of robot unicycle control vector inputs (dxu). Recieved a current robot pose input array (x) of size " + num2str(size(x,1)) + "x" + num2str(size(x,2)) + "and unicycle control vector input array (dxu) of size " + num2str(size(dxu,1)) + "x" + num2str(size(dxu,2)) + ".");  


        N = size(dxu, 2);
        
        if(N < 2)
           return 
        end
        
        %Shift to single integrator domain
        xi = uni_si_states(x);
        dxi = uni_si_dyn(dxu, x);
               
        % Normalize velocities
        norms = arrayfun(@(idx) norm(dxi(:, idx)), 1:N);
        to_normalize = norms > magnitude_limit;
        if(size(to_normalize, 2) > 0)
            dxi(:, to_normalize) = magnitude_limit*dxi(:, to_normalize)./norms(to_normalize);
        end
        
        dxi_safe = si_barrier_certificate(dxi, xi);
        
        %Set robot velocities to new velocities
        dxu = si_uni_dyn(dxi_safe, x);

    end
end

