function hullInd = ConvexHullInd(points)
numPoints=size(points,2);
hullInd=[];
[~, startIdx]=min(points(1,:)); % Finding the leftmost point
currentIdx=startIdx;
hullPoints=[];
repeat=true;
while repeat
    hullPoints(end+1)=currentIdx; % Adding index to hull points
    nextIdx=mod(currentIdx,numPoints)+1; % Starting from the next point
    for i = 1:numPoints
        if i == currentIdx
            continue;
        end
        % Using cross product to check if it turn left, wraping the hull
        direction = det([points(:,nextIdx)-points(:,currentIdx), points(:,i)-points(:,currentIdx)]);
        if direction > 0
            nextIdx = i;
        end
    end
    currentIdx = nextIdx;
    % Checking if we have wrapped around to the first hull point
    if nextIdx == startIdx
        repeat = false;
    end
end
% Adding the first hull point to the end of the array to get wrapped convex
% hull
hullInd = [hullPoints hullPoints(1)];
end