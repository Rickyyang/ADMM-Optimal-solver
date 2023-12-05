% constraint for local velocity constraint for one agent
function constraint = constraintLocalVelocity(n_agent,traj_length,velocityCap,timePeriod,agentID)
% total number of agent is needed for multi-agent case 

if timePeriod(2)>traj_length
    timePeriod(2) = traj_length;
end
zDim = [2,diff(timePeriod)]';
constraint.intermediateVariableDim = zDim;  %dimension of z (before vectorization)
constraint.equalityConstrainDx =@trajectoryEdgeLocal;  % equality constrain D(x) = z;
constraint.equalityConstrainDer =@trajectoryEdgeLocal_der;
constraint.intermediateVariableUpdateFcn =@(z,x) sephereProject(z,velocityCap);
constraint.dov = @(z,x) velocityCap-cnorm(reshape(z,2,[]));
constraint.type =@(x) 'ineq';

    function out = trajectoryEdgeLocal_der(~)
        diag = eye(diff(timePeriod));
        ins = zeros(diff(timePeriod),1);
        outLocal = [ins,diag]-[diag,ins];
        out = zeros(diff(timePeriod),traj_length*n_agent);
        local_constraint_startID = traj_length*(agentID-1)+timePeriod(1);
        local_constraint_endID = traj_length*(agentID-1)+timePeriod(2);
        out(:,local_constraint_startID:local_constraint_endID)=outLocal;
        out = kron(out,eye(2));
    end

    function out = trajectoryEdgeLocal(x)
        x = reshape(x,2,[],n_agent);
        out = vec(diff(x(:,timePeriod(1):timePeriod(2),agentID),[],2));
    end
end

%% Generate the vectorized trajectory Edge Function and its derivative
    