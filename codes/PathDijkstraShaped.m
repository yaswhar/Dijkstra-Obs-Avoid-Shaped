function P = PathDijkstraShaped(dir)
% Reading the input text file
txtread=readmatrix(dir,"Range",[1 1]);
% Number of the obstacles' vertices, or in another words, the total
% number of the obstacles' sides.
numPoints=txtread(1,1);
% Storing the coordinates of the obstacles' vertices
obsverts=reshape(txtread(2:1+numPoints,:)',2,[]);
% Truncating the data matrix since I already stored the obstacles' data
start=txtread(2+numPoints,[1 2])';
final=txtread(3+numPoints,[1 2])';
txtread=txtread(4+numPoints:end,:);
% Number of the vertices of the  robot geometry
numVert=txtread(1,1);
% Storing the coordinates of the robot's vertices
vertices=reshape(txtread(2:1+numVert,[1 2])',2,[]);
obs={}; % Pre-allocating a cell for clustering the obstacles.
numVertx=size(obsverts,2);
startCol=1;
% Clustering the obstacles.
% If the first point is repeated again in the matrix, then from the start
% to that next repeatition index is for one obstacle, and then by
% truncating the matrix from start to that point, the same process is done
% to find the other obstacle.
while startCol <= numVertx-1
    for endCol= startCol + 1:numVertx
        if obsverts(:,startCol)==obsverts(:,endCol)
            obs{end+1}=obsverts(:,startCol:endCol);
            startCol=endCol;
            break;
        end
    end
    startCol=startCol+1;
end
numObs=size(obs,2);
% Finding the vectors from the robot's vertices to the start point which
% will be used for augmenting the obstacles using the Minkowski method
vectors=-vertices+start.*[1 1 1];
aug=cell(1,numObs);
for i=1:numObs
    % Augmentation of the obstacles using the Minkowski method
    augObsTmp=unique(reshape(repmat(obs{i},[1 1 numVert])+permute(repmat(vectors,[1 1 size(obs{i},2)]),[1 3 2]),2,[])','rows')';
    % Finding the convex hull formed by the new generated point by the
    % augmentation.
    k=ConvexHullInd(augObsTmp);
    aug{i}=augObsTmp(:,k);
end
% Storing the nodes of the graph
nodes=[start cell2mat(aug) final];
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
            [xtmp,ytmp]=polyxpoly(nodes(1,[i j]),nodes(2,[i j]),aug{k}(1,:),aug{k}(2,:),"unique");
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
            if size(tmp,2)>1
                [in,on]=inpolygon([tmp(1,:) mean(tmp(1,:),"all")],[tmp(2,:) mean(tmp(2,:),"all")],aug{k}(1,:),aug{k}(2,:));
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
animateRobotMotion(P,dist,nodes,vertices,start,final,aug,obs);
end

%% Plotting the movement of the robot

function animateRobotMotion(P, dist, nodes, vertices, start, final, aug, obstacles)
fig=figure;
axis equal;
hold on;
grid minor;
box on;
xlabel('X');
ylabel('Y');
title(['Robot Motion Animation. Minimum Distance: ' num2str(round(dist,3))]);
xlim([min([nodes(1,:),vertices(1,:)])-1,max([nodes(1,:),vertices(1,:)])+1]);
ylim([min([nodes(2,:),vertices(2,:)])-1,max([nodes(2,:),vertices(2,:)])+1]);
for i = 1:size(obstacles,2)
    plot(obstacles{i}(1,:),obstacles{i}(2,:),'k-','LineWidth',2);
    plot(aug{i}(1,:),aug{i}(2,:),'r--','LineWidth',0.5);
end
plot(P(1,:),P(2,:))
robotPlot=fill(vertices(1,:),vertices(2,:),'cyan');
scatter(start(1),start(2),'bo','filled');
scatter(final(1),final(2),'ro','filled');
pause(1);
for i = 1:length(P)-1
    stepVector=(P(:,i+1)-P(:,i))/50;
    for j = 1:50
        currentPos=get(robotPlot,'Vertices');
        newPos=currentPos'+stepVector.*[1 1 1];
        set(robotPlot, 'Vertices', newPos');
        pause(0.05);
        if ~ishghandle(fig)
            return;
        end
    end
end
end