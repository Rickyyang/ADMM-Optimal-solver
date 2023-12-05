function constraint = constraintVelocity(xStart,xEnd,xDim,velocityCap)
% total number of agent is needed for multi-agent case 
n_agent = size(xStart,2);
traj_length = xDim(2)/n_agent;
for i = 1:n_agent
    [D_i{i},dD_i{i}]= trajectoryEdge(xStart(:,i),xEnd(:,i));
end
% combine function
D = multiAgentFunctionIntegrate(D_i,traj_length,'vertical');
dD = multiAgentFunctionIntegrate(dD_i,traj_length,'diagonal');
% [D,dD]= trajectoryEdge(xStart,xEnd);
zDim = [xDim(1),(xDim(2)+n_agent)]';
constraint.intermediateVariableDim = zDim;  %dimension of z (before vectorization)
constraint.equalityConstrainDx = D;  % equality constrain D(x) = z;
constraint.equalityConstrainDer = dD;
constraint.intermediateVariableUpdateFcn =@(z,x) sephereProject(z,velocityCap);
constraint.dov = @(z,x) velocityCap-cnorm(reshape(z,2,[]));
constraint.type =@(x) 'ineq';
end