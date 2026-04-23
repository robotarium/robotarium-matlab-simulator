function R = rotation_matrix(theta)
            % ROTATIONMATRIX Returns a 2D rotation matrix for angle theta
            %
            %   R = ROTATIONMATRIX(THETA) returns a 2x2xN array of
            %   rotation matrices for each angle in THETA
            %
            %   Example:
            %       R = object.rotationMatrix([0, pi/4, pi/2])
            %
            %   Notes:
            %       Used in distance sensor simulation
            theta = theta(:)'; % ensure row vector, size 1×N
            c = cos(theta);
            s = sin(theta);

            % Preallocate 3-D array
            R = zeros(3, 3, length(theta));

            R(1,1,:) = c;
            R(1,2,:) = -s;
            R(2,1,:) = s;
            R(2,2,:) = c;
            R(3,3,:) = 1;
        end