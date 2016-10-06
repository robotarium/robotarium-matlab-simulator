%% random_connectedGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a random, connected GL with v verticies and (v-1) + e edges 
%% Example Usage 
%   L = random_connectedGL(4, 3);
%% Implementation
function [ L ] = random_connectedGL(v, e)

    L = zeros(v, v); 

    for i = 2:v
        
        edge = randi(i-1, 1, 1);

        %Update adjancency relations
        L(i, edge) = -1;
        L(edge, i) = -1;
        
        %Update node degrees
        L(i, i) = L(i, i) + 1; 
        L(edge, edge) = L(edge, edge) + 1;
    end
    
    %This works becuase all nodes have at least 1 degree.  Choose from only
    %upper diagonal portion 
    potEdges = find(triu(bsxfun(@xor, L, 1)) == 1); 
    sz = size(L);
    
    numEdges = min(e, length(potEdges)); 
    
    if (numEdges <= 0)
       return 
    end
    
    %Indices of randomly chosen extra edges
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

