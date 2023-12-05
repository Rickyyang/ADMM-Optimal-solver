% project one point to one trajectory
function [projection_vertex_id, projection_percentage,min_dis] = trajectoryProject(fix_point,x)
l = size(x,2);
projection_vertex = zeros(2,l-1);
projection_percentage_all = zeros(1,l-1);
for i = 1:l-1
    [projection_vertex(:,i),projection_percentage_all(i)] = segmentProject(fix_point,x(:,i),x(:,i+1));
end
p = fix_point*ones(1,l-1);
dis = sum((projection_vertex-p).^2);
[min_dis,projection_vertex_id] = min(dis);
projection_percentage = projection_percentage_all(projection_vertex_id);
end