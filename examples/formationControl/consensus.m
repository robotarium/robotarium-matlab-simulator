for i = 1:N

    % Initialize velocity to zero for each agent.  This allows us to sum
    % over agent i's neighbors
    dx(:, i) = [0 ; 0];

    % Get the topological neighbors of agent i based on the graph
    % Laplacian L
    neighbors = r.getTopNeighbors(i, L);

    % Iterate through agent i's neighbors
    for j = neighbors

        %%% CONSENSUS %%%

        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        dx(:, i) = dx(:, i) + (x(1:2, j) - x(1:2, i));

        %%% END CONSENSUS %%%
    end      
end