%% calculate the distance between a point to a line defined by two points

function Dis = p2lineDistance(x,normalVec,Dist2Origin)
%calaulate the projection of point p on line x1 x2
Dis = normalVec'*x - Dist2Origin;
end