%% return the distance and side for each way points on the trajectory with 
%  respect to the normal vector
function [f,df] = traj2HalfPlaneDistance(normalVec,dis2Origin)
% convert normal vector into a unit vector
if norm(normalVec) ~= 1
    normalVec = normalVec/norm(normalVec);
    dis2Origin = dis2Origin/norm(normalVec);
end
%generate functions
    function dis = fcntraj2Line(x)
        dis = (normalVec'*x - dis2Origin)';
    end
    function disDer = fcntraj2LineDer(x)
        disDer = kron(eye(size(x,2)),normalVec');        
    end
f = @fcntraj2Line;
df = @fcntraj2LineDer;
end

