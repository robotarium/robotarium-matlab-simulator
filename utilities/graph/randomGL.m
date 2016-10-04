%% randomGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a random grab laplacian with a specified number of verticies and
% edges 
%% Example Usage 
%   L = randomGL(5, 3);
%% Implementation
function [ L ] = randomGL(v, e)

    L = tril(ones(v, v));
    
    %This works becuase I can't select diagonals
    potEdges = find(triu(L) == 0);
    sz = size(L); 
    
    %Rest to zeros
    L = L - L;
   
    numEdges = min(e, length(potEdges)); 
    edgeIndices = randperm(length(potEdges), numEdges);
    
    for index = edgeIndices 
        
       [i, j] = ind2sub(sz, potEdges(index));
        
        %Update adjacency relation
        L(i, j) = -1;
        L(j, i) = -1; 
        
        %Update degree relation
        L(i, i) = L(i, i) + 1;
        L(j, j) = L(j, j) + 1;
    end
end

