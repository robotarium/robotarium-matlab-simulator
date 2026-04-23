classdef UnicycleEKF < handle
    % UNICYCLEEKF 
    % An Extended Kalman Filter for a differential drive unicycle robot.
    
    properties
        state % [x; y; theta]
        P     % Covariance matrix
        b     % Base length
        r     % Wheel radius
        M     % Encoder noise matrix
        Q     % Process noise matrix
    end
    
    methods
        function obj = UnicycleEKF(initial_state, initial_covariance, b, r, M, Q)
            obj.state = initial_state(:);
            obj.P = initial_covariance;
            obj.b = b;
            obj.r = r;
            obj.M = M;
            obj.Q = Q;
        end
        
        function predict(obj, v, w, dt)
            theta = obj.state(3);
            
            % 1. State prediction
            obj.state(1) = obj.state(1) + v * cos(theta) * dt;
            obj.state(2) = obj.state(2) + v * sin(theta) * dt;
            obj.state(3) = obj.state(3) + w * dt;
            
            % Normalize theta between -pi and pi
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
            
            % 2. State transition Jacobian (F)
            F = eye(3);
            F(1, 3) = -v * sin(theta) * dt;
            F(2, 3) =  v * cos(theta) * dt;
            
            % 3. Input noise mapping
            % Map wheel velocities [w_r; w_l] to [v; w]
            J_u = [obj.r/2,  obj.r/2; 
                   obj.r/obj.b, -obj.r/obj.b];
               
            % Map [v; w] to state space [x; y; theta]
            V = [cos(theta)*dt, 0; 
                 sin(theta)*dt, 0; 
                 0,             dt];
                 
            % Combined input Jacobian
            W = V * J_u;
            
            % 4. Covariance prediction
            obj.P = F * obj.P * F' + W * obj.M * W' + obj.Q;
        end
        
        function update_gps(obj, z, R)
            H = [1, 0, 0; 
                 0, 1, 0];
             
            y = z(:) - H * obj.state;
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            obj.state = obj.state + K * y;
            obj.P = (eye(3) - K * H) * obj.P;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
        end
        
        function update_orientation(obj, z, R)
            H = [0, 0, 1];
            
            y = z - H * obj.state;
            y = atan2(sin(y), cos(y)); % Normalize angle error
            
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            obj.state = obj.state + K * y;
            obj.P = (eye(3) - K * H) * obj.P;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
        end
        
        function update_gyro(obj, yaw_rate, dt, R)
            % Treat the gyro as a pseudo-measurement of the change in yaw
            H = [0, 0, 1];
            
            y = yaw_rate * dt; 
            R_effective = R * dt^2;
            
            S = H * obj.P * H' + R_effective;
            K = obj.P * H' / S;
            
            obj.state = obj.state + K * y;
            obj.P = (eye(3) - K * H) * obj.P;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
        end
    end
end