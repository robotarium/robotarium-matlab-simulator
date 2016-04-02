function [ L ] = randomGL(v, e)
%RANDOMGL Outputs a randomly generated, undirected, connected graph Laplacian with 'n'
%nodes
%   Detailed explanation goes here

    L = zeros(v, v); 

    for i = 2:v
        
        edge = randi(i-1, 1, 1);

        %Update adjancency relations
        L(i, edge) = 1;
        L(edge, i) = 1;
        
        %Update node degrees
        L(i, i) = L(i, i) + 1; 
        L(edge, edge) = L(edge, edge) + 1;
    end
    
    %This works becuase all nodes have at least 1 degree
    potEdges = find(L == 0); 
    sz = size(L);

    for node = 1:min([(e - (v - 1)) length(potEdges)/2]);
        index = randi(length(potEdges), 1, 1);
        [i, j] = ind2sub(sz, potEdges(index));
        
        %Update adjacency relation
        L(i, j) = 1;
        L(j, i) = 1; 
        
        %Update degree relation
        L(i, i) = L(i, i) + 1;
        L(j, j) = L(j, j) + 1;
        
        %Get rid of choice
        potEdges(index) = [];   
        potEdges(find(potEdges == sub2ind(sz, j, i))) = [];
    end
end

