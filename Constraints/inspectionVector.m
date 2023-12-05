% return the distance function between agents
function [inspection_vector_fcn, inspection_vector_grad_fcn] = inspectionVector(inspection_constraints,traj_length,n_agent,varargin)
% if ~isempty(varargin)
%     k_robust = varargin{1};
% else 
%     k_robust = [];
% end

inspection_vector_fcn = @(x) InspectionDistance(x,inspection_constraints,traj_length,n_agent,1);%,k_robust);
inspection_vector_grad_fcn = @(x) InspectionDistance(x,inspection_constraints,traj_length,n_agent,2);%,k_robust);
end

function out = InspectionDistance(x,inspection_constraints,traj_length,n_agent,out_type)%,k_robust)
% function calculate the inspection when out_type = 1 or its derivative out_type = 2
x = reshape(x,2,[],n_agent);
n_pair = size(inspection_constraints,2); % number of inspection pairs
inspection_pair = zeros(2,n_pair); % inspection pairs
inspection_time = zeros(2,n_pair); 
inspection_duration = zeros(1,n_pair);
TrajDis = [];
TrajDis_grad = [];

for i = 1:n_pair
    inspection_pair(:,i) = inspection_constraints(i).agentPair';
    inspection_time(:,i) = inspection_constraints(i).meetTime;
    inspection_duration(i) = inspection_constraints(i).duration;
    
    if inspection_duration(i) == 0
        error('Inspection duration could not be zero!')
    end
end

for i = 1:n_pair
    agent1 = inspection_pair(1,i);
    agent2 = inspection_pair(2,i);
    inspection_end_time_agent1 = min(traj_length,inspection_time(1,i)+inspection_duration(i)-1);
    inspection_end_time_agent2 = min(traj_length,inspection_time(2,i)+inspection_duration(i)-1);
    inspection_duration_i = min(inspection_end_time_agent1-inspection_time(1,i)+1,inspection_end_time_agent2-inspection_time(2,i)+1);
    inspection_end_time_agent1 = inspection_time(1,i)+inspection_duration_i-1;
    inspection_end_time_agent2 = inspection_time(2,i)+inspection_duration_i-1;
    if out_type == 1
        D = x(:,inspection_time(1,i):inspection_end_time_agent1,agent1)-x(:,inspection_time(2,i):inspection_end_time_agent2,agent2);
        TrajDis = [TrajDis,D];
    elseif out_type ==2 % asking for gradient
        inspection_start_id = [(agent1-1)*traj_length+inspection_time(1,i);(agent2-1)*traj_length+inspection_time(2,i)];
        inspection_end_id =  [(agent1-1)*traj_length+inspection_end_time_agent1;(agent2-1)*traj_length+inspection_end_time_agent2];
        TrajDis_grad_i = zeros(inspection_duration_i,traj_length*n_agent);
        TrajDis_grad_i(:,inspection_start_id(1):inspection_end_id(1)) = eye(inspection_duration_i);
        TrajDis_grad_i(:,inspection_start_id(2):inspection_end_id(2)) = -eye(inspection_duration_i);
        TrajDis_grad = [TrajDis_grad;TrajDis_grad_i];    
    end
end

if out_type == 1
    TrajDis = vec(TrajDis);
    out = TrajDis;
elseif out_type == 2
    TrajDis_grad = kron(TrajDis_grad,eye(2));
    out = TrajDis_grad;
end

end
        