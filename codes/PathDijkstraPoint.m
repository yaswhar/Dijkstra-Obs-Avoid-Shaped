function P = PathDijkstraPoint(dir)
% Reading the input text file
txtread=readmatrix(dir,"Range",[1 1]);
% Number of the obstacles' vertices, or in another words, the total
% number of the obstacles' sides.
numPoints=txtread(1,1);
% Storing the coordinates of the obstacles' vertices
obsverts=reshape(txtread(2:1+numPoints,:)',2,[]);
obs={}; % Pre-allocating a cell for clustering the obstacles.
numVert=size(obsverts,2);
startCol=1;
% Clustering the obstacles.
% If the first point is repeated again in the matrix, then from the start
% to that next repeatition index is for one obstacle, and then by
% truncating the matrix from start to that point, the same process is done
% to find the other obstacle.
while startCol <= numVert-1
    for endCol= startCol + 1:numVert
        if obsverts(:,startCol)==obsverts(:,endCol)
            obs{end+1}=obsverts(:,startCol:endCol);
            startCol=endCol;
            break;
        end
    end
    startCol=startCol+1;
end
numObs=size(obs,2);
start=txtread(end-1,[1 2])';
final=txtread(end,[1 2])';
% Storing the nodes of the graph
nodes=[start unique(obsverts',"rows")' final];
AdjMat=zeros(size(nodes,2));
flag=1;
% Findig the Adjacency Matrix for representing the graph
% In this matrix, a_{ij} represents the distance between the i-th node and
% j-th node. If it's zero, then there is no edge between those nodes. Also
% since the edges do not have direction, the matrix is symmetric having
% zeros on its diagonal.
for i=1:size(nodes,2)-1
    for j=i+1:size(nodes,2)
        for k=1:numObs
                        % Finding the intersection points of the augmented obstacle and
            % the edge betweem the i-th node and j-th
            [xtmp,ytmp]=polyxpoly(nodes(1,[i j]),nodes(2,[i j]),obs{k}(1,:),obs{k}(2,:),"unique");
            tmp=[xtmp';ytmp'];
                        % If it has more than one intersection points, then it can be
            % one of these cases. Case 1: It goes through to obstacle,
            % which is not allowable. Case 2: It is one side of the
            % augmented obstacle which is allowable. To find this, I take
            % the mean of the intersection points, and find whether the
            % intesection points and their mean is inside or on the shape of
            % obstacle. If any of them is strictly inside the shape, then
            % it means that the corresponding edge passes through the
            % augmented obstacle.
            if  size(tmp,2)>1
                [in,on]=inpolygon([tmp(1,:) mean(tmp(1,:),"all")],[tmp(2,:) mean(tmp(2,:),"all")],obs{k}(1,:),obs{k}(2,:));
                if any(in&~on)
                    flag=0;
                    break
                end
            end
        end
        if flag
            AdjMat(i,j)=norm(nodes(:,i)-nodes(:,j));
            AdjMat(j,i)=norm(nodes(:,i)-nodes(:,j));
        end
        flag=1;
    end
end
% Finding the shortest path in Adjacency Matrix using Dijkstra Algorithm
[P,dist]=dijkstraAdjMat(AdjMat,1,size(nodes,2));
P=nodes(:,P);
% Plotting the results
figure;
hold on;
grid off;
box on;
axis equal;
xlabel('X');
ylabel('Y');
xlim([min(nodes(1,:))-1 max(nodes(1,:))+1]);
ylim([min(nodes(2,:))-1 max(nodes(2,:))+1]);
title(['Path found by Dijkstra (Minimum Distance = ', num2str(round(dist,3)) ,')']);
scatter(start(1), start(2),'b','filled');
scatter(final(1), final(2),'b','filled');
for i = 1:numObs
    plot(obs{i}(1,:),obs{i}(2,:),'color','red','LineWidth',1.5);
end
plot(P(1,:),P(2,:),"LineStyle","--")
end