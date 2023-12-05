function InitialTrajectoryGeneration(map_size,n_agent,xStart,xEnd,t_max,velocityCap,costFcn,costFcnDer,filename)
%% traj initialization
x_initial = initTrajGen(xStart,xEnd,t_max,velocityCap);
xDim = size(x_initial);
%% basic constrains
% velocity constraint
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);

% map edge constraint
mapEdgeNormVec =  [1,0,-1,0;... 
                0,1, 0,-1];  
mapEdgeDis2Orig = [0,0,-map_size(1),-map_size(2)];
stayInTheMap = constraintSafeZone(mapEdgeNormVec,mapEdgeDis2Orig,2);
% basicConstraintFcn =@(xDim) {velocityConstraint(xDim),stayInTheMap};
% basicConstraint = basicConstraintFcn(xDim);
constraint = combineConstraints(stayInTheMap,velocityConstraint);
%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.05;
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
%% make combine all info send to admm solver
problem.x_initial=x_initial;
problem.flagCustomSolver = false;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
[x,output] = admm_solver(problem,'debug','maxfminit',4);
% save function
% fileNameSave=[mfilename regexprep(char(datetime('now')),'[: ]+','-')];
% save(fileNameSave)
%% with safezone constraint
y = reshape(x,2,[],n_agent);
for i = 1:n_agent
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end
% uncertainty_map = uncertaintyMap(x);
% % figure()
% meshz(uncertainty_map)
save(filename)