function [ L ] = randomGL(v, e)
%RANDOMGL Outputs a randomly generated, undirected, connected graph Laplacian with 'n'
%nodes
%   Detailed explanation goes here

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

