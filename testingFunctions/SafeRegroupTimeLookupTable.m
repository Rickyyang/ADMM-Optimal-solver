clear
load('results\FinalResult.mat')

ellipse2SafezoneConstraint =@(timePeriod) constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_s,dis2Origin_s,1,zoneType,'agents_involved',1);
lookupTable=zeros(1,traj_length+1);
reference=struct('start',{},'goal',{},'x_ref',{},'LookupTable',{});
for i = 1:n_agent
    team_traj = [xStart(:,i),y(:,:,i),xEnd(:,i)];
    %find look up table
    for t = 1:(traj_length+1)
        for r = (t+1):(traj_length+2)
            dis=ellipse2SafezoneConstraint([t,r]).equalityConstrainDx(team_traj);
            if all(dis>=0)
                lookupTable(t)=r;
            else
                break
            end
        end
    end
    % write reference file
    agent_i_ref.start=xStart(:,i);
    agent_i_ref.goal=xEnd(:,i);
    agent_i_ref.x_ref=[y(:,:,i),xEnd(:,i)];
    agent_i_ref.LookupTable=lookupTable;
    reference(i)=agent_i_ref;
    % clear result
    lookupTable=zeros(1,traj_length+1);
end

txt=jsonencode(reference);
fileID = fopen('PlanningResultReference.json','w');
fprintf(fileID,'%s',txt);

