function min_edge_index = findInspectionVerticesPair(x,traj_length,inspection_constraints)
%extract information structure "inspection_constraints"
n_pair = size(inspection_constraints,2); % number of inspection pairs
inspection_pair = zeros(2,n_pair); % inspection pairs
inspection_time = inspection_pair; % obscure inspection time +/- 3 time steps
time_slack = zeros(1,n_pair);
for i = 1:n_pair
    inspection_pair(:,i) = inspection_constraints(i).agentPair';
    inspection_time(:,i) = inspection_constraints(i).meetTime';
    time_slack(i) = inspection_constraints(i).slack;
end
min_edge_index=zeros(2,n_pair);
for i = 1:n_pair
    slack = time_slack(i);
    agent1_time = inspection_time(1,i);
    agent2_time = inspection_time(2,i);
    agent1 = inspection_pair(1,i);
    agent2 = inspection_pair(2,i);
    if slack ~= 0
        % start around the inspection time, 'max' used for cases inspection
        % time minus slack is smaller than the start time
        a1_start_id = max((agent1-1)*traj_length+agent1_time - slack,(agent1-1)*traj_length+1);
        a1_end_id = min((agent1-1)*traj_length+agent1_time + slack,agent1*traj_length);
        a2_start_id = max((agent2-1)*traj_length+agent2_time - slack,(agent2-1)*traj_length+1);
        a2_end_id = min((agent2-1)*traj_length+agent2_time + slack,agent2*traj_length);
        a1 = x(:,a1_start_id:a1_end_id);
        a2 = x(:,a2_start_id:a2_end_id);
        D = euclideanDistMatrix(a1,a2);
        min_dis = min(D(:));
        [v1,v2]=find(D==min_dis);
        min_edge_index(:,i)=[a1_start_id+v1-1;a2_start_id+v2-1];
    else
        a1_id = (agent1-1)*traj_length+agent1_time;
        a2_id = (agent2-1)*traj_length+agent2_time;
        min_edge_index(:,i)=[a1_id;a2_id];
    end
end
end