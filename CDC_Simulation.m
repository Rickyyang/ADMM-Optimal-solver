close all
clc
clear

methodSolver='admm';
% 
map_size = [10,10];
%% two agent trajectory properities
n_agent = 3;
xStart=[0 0;10 0;5 0]';
xEnd=[10 10;4  5;5 10]';
t_max= 20;
velocityCap = 1;
Sigma = 1;
traj_length = t_max-1;

[costFcn,costFcnDer] = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma);

% InitialTrajectoryGeneration(map_size,n_agent,xStart,xEnd,t_max,velocityCap,costFcn,costFcnDer,savefile);
savefile = 'base_result.mat';
load(savefile)
% load('10x10_2_zones.mat')
% load('10x10_2Zone_1Observation.mat')
% increase time period
multiple = 2;
x = increase_traj_density(x,xStart,multiple);
traj_length = multiple*traj_length;

%% recreate cost function for new traj length
[costFcn,costFcnDer] = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma);
uncertaintyMap = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma,'uncertainty map');
xDim = size(x);
%% base constraints after resize
velocityCap = velocityCap/multiple;
velocityConstraint_new = constraintVelocity(xStart,xEnd,xDim,velocityCap);
basicConstraint = combineConstraints(stayInTheMap,velocityConstraint_new);
%% additional constraints 
%% safe zone specifications
normalVec_1 =  [1,0,-1,0;... %0,-1, 0;...
               0,1, 0,-1];  %1, 0,-1];
dis2Origin_1 = [6,3,-5,1]';% 5,-4,-3];
% safeZoneConstraint_1 = constraintSafeZone(normalVec_1,dis2Origin_1,1);

normalVec_2 =  [1,0,-1,0;... 
              0,1, 0,-1];
dis2Origin_2 = [2,7.5,-0.5,-6]';
% safeZoneConstraint_2 = constraintSafeZone(normalVec_2,dis2Origin_2,1);
safeZoneNormalVec = {normalVec_1,normalVec_2};
safeZonedis2Origin = {dis2Origin_1,dis2Origin_2};

%% meeting schedule
meetTime = [20;35];
agentPair = [1,2;1,3]';
% inspection constraint
for i = 1:length(meetTime)
    inspection_constraints{i}.agentPair = agentPair(:,i)';
    inspection_constraints{i}.duration = 1;
    inspection_constraints{i}.meetTime = meetTime(i);
    inspection_constraints{i}.inspectDistance = 0.1;
    inspectionConstraint{i}=constraintInspection(inspection_constraints{1},traj_length,n_agent);
end
% agent 1
timePeriod = [1;15];
zoneType = 'out';
ellipse2SafezoneConstraint{1} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_1,dis2Origin_1,n_agent,zoneType);
ellipse2SafezoneConstraint{2} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_2,dis2Origin_2,n_agent,zoneType);
% timePeriod = [15;25];
% zoneType = 'out';
% ellipse2SafezoneConstraint{3} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_1,dis2Origin_1,n_agent,zoneType);
% ellipse2SafezoneConstraint{4} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_2,dis2Origin_2,n_agent,zoneType);
% timePeriod = [25;traj_length];
% zoneType = 'out';
% ellipse2SafezoneConstraint{3} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_1,dis2Origin_1,n_agent,zoneType);
% ellipse2SafezoneConstraint{4} = constraintSafeZoneTimeSpecified(timePeriod,velocityCap,normalVec_2,dis2Origin_2,n_agent,zoneType);

%% with safezone constraint
constraint = combineConstraints(stayInTheMap,velocityConstraint_new,ellipse2SafezoneConstraint{:});%,inspectionConstraint{:});
constraint.safeZones.normals=safeZoneNormalVec;
constraint.safeZones.distances=safeZonedis2Origin;
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
problem.x_initial=x;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.uncertaintyMap = uncertaintyMap;
problem.constraint = constraint;
problem.penalty = penalty;
[x,output] = admm_solver(problem,'debug','maxfminit',4);
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
