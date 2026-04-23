function L = lineGL(n)
% LINEGL  Return the graph Laplacian of a path (line) graph on n nodes.
%
%   In a path graph interior nodes have degree 2 and the two endpoint
%   nodes have degree 1.  The Laplacian is tridiagonal with 2s on the
%   main diagonal and -1s on the super- and sub-diagonals, except for the
%   (1,1) and (n,n) entries which are set to 1.
%
%   Input
%   -----
%   n : number of nodes (positive integer, n >= 2)
%
%   Output
%   ------
%   L : nxn graph Laplacian
%
%   Example
%   -------
%       L = lineGL(5);

    % Build the tridiagonal structure assuming all interior degrees first.
    L = 2*eye(n) - diag(ones(1, n-1), 1) - diag(ones(1, n-1), -1);

    % Correct the two endpoint degrees from 2 to 1.
    L(1, 1) = 1;
    L(n, n) = 1;
end
