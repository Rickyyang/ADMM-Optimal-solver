clc
clear
load('base_result.mat')
% load('10x10_2_zones.mat')
% load('10x10_2Zone_1Observation.mat')
% increase time period
multiple = 1;
x = increase_traj_density(x,xStart,multiple);
traj_length = multiple*traj_length;

%% recreate cost function for new traj length
[costFcn,costFcnDer] = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma);
uncertaintyMap = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma,'uncertainty map');
xDim = size(x);
basicConstraint = basicConstraintFcn(xDim);
%% additional constraints 
% inspection constraint
inspection_constraints.agentPair = [1,2];
inspection_constraints.duration = 1;
inspection_constraints.meetTime = 20;
inspection_constraints.inspectDistance = 0.1;
inspectionConstraint=constraintInspection(inspection_constraints,traj_length,n_agent);

inspection_constraints_2.agentPair = [1,3];
inspection_constraints_2.duration = 1;
inspection_constraints_2.meetTime = 35;
inspection_constraints_2.inspectDistance = 0.1;
inspectionConstraint_2=constraintInspection(inspection_constraints_2,traj_length,n_agent);
% fix point constraint
fixPoint = [7,7];
fixPointConstraint = constraintFixPoints(fixPoint,n_agent);
% safe zone constraint
normalVec_1 =  [1,0,-1,0;... %0,-1, 0;...
              0,1, 0,-1];  %1, 0,-1];
dis2Origin_1 = [8,4,-6,1];% 5,-4,-3];
safeZoneConstraint_1 = constraintSafeZone(normalVec_1,dis2Origin_1,1);

normalVec_2 =  [1,0,-1,0;... 
              0,1, 0,-1];
dis2Origin_2 = [2,7.5,-0.5,-6];
safeZoneConstraint_2 = constraintSafeZone(normalVec_2,dis2Origin_2,1);
safeZoneNormalVec = {normalVec_1,normalVec_2};
safeZonedis2Origin = {dis2Origin_1,dis2Origin_2};

%% with safezone constraint
constraint = combineConstraints(basicConstraint{:},safeZoneConstraint_1,safeZoneConstraint_2,inspectionConstraint);%,inspectionConstraint_2,fixPointConstraint);
constraint.safeZones.normals=safeZoneNormalVec;
constraint.safeZones.distances=safeZonedis2Origin;
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = penalty_value; %*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
problem.x_initial=x;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.uncertaintyMap = uncertaintyMap;
problem.constraint = constraint;
problem.penalty = penalty;
[x,output] = admm_solver(problem,'debug');
figure()
y = reshape(x,2,[],n_agent);
for i = 1:n_agent
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end
plotSafeZone(safeZoneNormalVec,safeZonedis2Origin)
uncertainty_map = uncertaintyMap(x);
figure()
meshz(uncertainty_map)
fileNameSave=[mfilename regexprep(char(datetime('now')),'[: ]+','-')];
save(fileNameSave)
