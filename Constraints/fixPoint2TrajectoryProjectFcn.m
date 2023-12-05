function [projected_point,projected_point_grad] = fixPoint2TrajectoryProjectFcn(fix_point,x,nAgent)
lx = size(x,2);
lSingleAgent = lx/nAgent;
projected_vertex = zeros(2,(lSingleAgent-1)*nAgent);        % fix points' projection on each segments
projected_percentage_all = zeros(1,(lSingleAgent-1)*nAgent);    
j = 1;
for i = 1:lx-1
    if rem(i,lSingleAgent)==0
        continue
    end
    [projected_vertex(:,j),projected_percentage_all(j)] = segmentProject(fix_point,x(:,i),x(:,i+1));
    j=j+1;
end
p = fix_point*ones(1,(lSingleAgent-1)*nAgent);
dis = sum((projected_vertex-p).^2);
[min_dis,projection_vertex_id] = min(dis);
projected_point = projected_vertex(:,projection_vertex_id); % returen the cloestest projected point with respect to the fix point
projected_percentage = projected_percentage_all(projection_vertex_id);
% init the gradiant of the projection function
projected_point_grad = zeros(2,lx*2);
end_id = (projection_vertex_id+1)*2;
start_id = end_id - 3;
projected_point_grad(:,start_id:end_id)=kron([projected_percentage 1-projected_percentage],eye(2));
end