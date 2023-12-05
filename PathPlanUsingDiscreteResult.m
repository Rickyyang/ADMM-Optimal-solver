close all
clc
clear

methodSolver='admm';
% 
map_size = [6,6];
%% two agent trajectory properities
n_agent = 3;
xStart=[0 0;2 0;5 0]';
xEnd=[5 5;3 5;2 2]';
t_max= 12;
velocityCap = 0.7;
Sigma = 1;
traj_length = t_max-1;

savefile = 'base_result.mat';


[costFcn,costFcnDer] = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma);
% InitialTrajectoryGeneration(map_size,n_agent,xStart,xEnd,t_max,velocityCap,costFcn,costFcnDer,savefile);
load(savefile)
load('DiscreteResult.mat')
x = traj_init;
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
normalVec_1 =  [1,0,-1,0;...
               0,1, 0,-1];  
dis2Origin_1 = [4,3,-3,0]';
safeZoneConstraint_1 = constraintSafeZone(normalVec_1,dis2Origin_1,1);

safeZoneNormalVec = {normalVec_1};%{normalVec_1,normalVec_2};
safeZonedis2Origin = {dis2Origin_1};%{dis2Origin_1,dis2Origin_2};

%% meeting schedule
load('results\FinalResult.mat')
penalty_value = 1;
% Agent 1: at time 4, 16        : meet agent 2 
% agent 2: at time 4, 16, 20    : meet agent 1, anent 1, agent 3
% agent 3: at time 20 10        : meet agent 2, location [5.5;4]
meetTime = [4;16;16];
agentPair = [1,2;1,2;2,3]';

% meetTime = [18;20];
% agentPair = [1,2;2,3]';
% inspection constraint
for i = 1:length(meetTime)
    inspection_constraints(i).agentPair = agentPair(:,i)';
    inspection_constraints(i).duration = 2;
    inspection_constraints(i).meetTime = meetTime(i);
    inspection_constraints(i).inspectDistance = 0.1;
    inspectionConstraint{i}=constraintInspection(inspection_constraints(i),traj_length,n_agent);
end
%% avoid Point
avoid_point = [1;5.5];
timePeriod = [1;meetTime(1)];
ellipse2pointConstraint{1} = constraintEllipise(timePeriod,velocityCap,avoid_point,n_agent);
timePeriod = [meetTime(1);meetTime(2)];
ellipse2pointConstraint{2} = constraintEllipise(timePeriod,velocityCap,avoid_point,n_agent);
timePeriod = [meetTime(2);meetTime(3)];
ellipse2pointConstraint{3} = constraintEllipise(timePeriod,velocityCap,avoid_point,n_agent);
%% avoid region
%region specification
normalVec_s =  [1,0,-1,0;... %0,-1, 0;...
               0,1, 0,-1];  %1, 0,-1];
dis2Origin_s = [4,3,-3,0]';% 5,-4,-3];
% for agent 1
timePeriod_zone{1} = [1,4;4,16;16,traj_length]';
zoneType = 'out';
for i = 1:size(timePeriod_zone{1},2)
    timePeriod_i = timePeriod_zone{1}(:,i);
    ellipse2SafezoneConstraint{i} = constraintSafeZoneTimeSpecified(timePeriod_i,velocityCap,normalVec_s,dis2Origin_s,n_agent,zoneType,'agents_involved',1);
end
i1 = i;
% for agent 2
timePeriod_zone{2} = [1,4;4,16;16,20;20,traj_length]';
for i = 1:size(timePeriod_zone{2},2)
    timePeriod_i = timePeriod_zone{2}(:,i);
    ellipse2SafezoneConstraint{i+i1} = constraintSafeZoneTimeSpecified(timePeriod_i,velocityCap,normalVec_s,dis2Origin_s,n_agent,zoneType,'agents_involved',2);
end
i1 = i1+i;
% for agent 3
timePeriod_zone{3} = [1,8;8,16;16,traj_length]';
for i = 1:size(timePeriod_zone{3},2)
    timePeriod_i = timePeriod_zone{3}(:,i);
    ellipse2SafezoneConstraint{i+i1} = constraintSafeZoneTimeSpecified(timePeriod_i,velocityCap,normalVec_s,dis2Origin_s,n_agent,zoneType,'agents_involved',3);
end

%% with safezone constraint
constraint = combineConstraints(stayInTheMap,velocityConstraint_new,safeZoneConstraint_1,inspectionConstraint{:});%,ellipse2pointConstraint{:},ellipse2SafezoneConstraint{:});%,ellipse2SafezoneConstraint{:});%,inspectionConstraint{:});
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
[x,output] = admm_solver(problem,'debug','maxfminit',3);
figure()
y = reshape(x,2,[],n_agent);
fileNameSave=[mfilename regexprep(char(datetime('now')),'[: ]+','-')];
% save(fileNameSave)

timePeriod_zone{1} = [1,4;4,16;16,traj_length]';
timePeriod_zone{2} = [1,4;4,16;16,20;20,traj_length]';
timePeriod_zone{3} = [1,8;8,16;16,traj_length]';

for i = 1:n_agent
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end
plotSafeZone(safeZoneNormalVec,safeZonedis2Origin)
plotSafeZone({normalVec_s},{dis2Origin_s})
x = reshape(x,2,[],n_agent);
for i = 1:n_agent
    time = timePeriod_zone{i};
    for j = 1:size(time,2)
        focus1 = x(:,time(:,j),i);
        a = diff(time(:,j))*velocityCap/2;
        ellipseDraw(focus1,a);
        plotPoints(focus1,'bo')
    end
end
uncertainty_map = uncertaintyMap(x);
axis([0 6 0 6])
grid on
figure()
meshz(uncertainty_map)

