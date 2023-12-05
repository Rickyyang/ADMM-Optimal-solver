function [ProjectVertexId, ProjectPercentage] = multiTrajectoryProject(x,fix_points,n_agent,traj_length)
n = size(fix_points,2); %number of fix points need to pass
ProjectVertexId = zeros(1,n);
ProjectPercentage = zeros(1,n);
for i = 1:n
    fix_point = fix_points(i);
    ProjectVertexIdPi = zeros(1,n_agent);
    ProjectPercentagePi = zeros(1,n_agent);
    minDisPi = zeros(1,n_agent);
    for j = 1:n_agent
        start_id = (j-1)*traj_length+1;
        end_id = j*traj_length;
        x_i = x(:,start_id:end_id);
        [ProjectVertexId, ProjectPercentage,minDis] = trajectoryProject(x_i,fix_point);
        ProjectVertexIdPi(j) = ProjectVertexId;
        ProjectPercentagePi(j) = ProjectPercentage;
        minDisPi(j)= minDis;
    end
    [min_dis,closesetAgentId]= min(minDisPi);
    ProjectVertexId(i) = ProjectVertexIdPi(closesetAgentId);
    ProjectPercentage(i) = ProjectPercentagePi(closesetAgentId);
end
end