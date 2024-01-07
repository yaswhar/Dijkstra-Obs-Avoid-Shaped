function [shortestPath,totalCost]=dijkstraAdjMat(adjMat,startNode,endNode)
numNodes=size(adjMat,1);
unvisited=true(1,numNodes); % All nodes are initially unvisited
% Initial distance from start to each node is infinite
dist=inf(1,numNodes);
% Array for storing the previous node visited in the shortest path,
% initialized with -1, which mean no previous node has been visited
prev=-1*ones(1,numNodes);
dist(startNode)=0; % Distance from start to itself is always 0
while any(unvisited)
    % Finding the node with the smallest distance
    nextNode=find(unvisited & dist==min(dist(unvisited)),1);
    unvisited(nextNode)=false; % Now marking it as visited
    % Iterating over neighbors
    for neighbor=find(adjMat(nextNode,:) & unvisited)
        newDist=dist(nextNode)+adjMat(nextNode,neighbor);
        if newDist<dist(neighbor) % Checking if the new path is shorter
            dist(neighbor)=newDist;
            prev(neighbor)=nextNode; % Updating previous node
        end
    end
end
% Tracing the shortest path from end node to start node
% If still the value of the end node is infinite, then no shortest path is
% found.
if isinf(dist(endNode))
    shortestPath=[];
    totalCost=inf;
    disp('No path exists between the selected nodes');
else
    totalCost=dist(endNode);
    shortestPath=endNode;
    while prev(shortestPath(1))>0
        shortestPath=[prev(shortestPath(1)),shortestPath];
    end
end
end