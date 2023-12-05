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
t_max= 40;
r_max = 0.5;
velocityCap = 0.5;
Sigma = 1;
traj_length = t_max-1;

[costFcn,costFcnDer] = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma);
uncertaintyMap = mapUncertaintyModel(map_size,traj_length,'agent number',n_agent,'sigma',Sigma,'uncertainty map');

%% traj initialization
x_initial = initTrajGen(xStart,xEnd,t_max,r_max);
% x_initial = randomTrajGen(xStart,xEnd,t_max,r_max);
xDim = size(x_initial);
% save data for later use
save('agentProperities.mat','n_agent','traj_length','xStart','xEnd','map_size','uncertaintyMap','costFcn','costFcnDer','uncertaintyMap');
%% basic constrains
% velocity constraint
velocityConstraint = @(xDim) constraintVelocity(xStart,xEnd,xDim,velocityCap);

% map edge constraint
mapEdgeNormVec =  [1,0,-1,0;... 
                0,1, 0,-1];  
mapEdgeDis2Orig = [0,0,-map_size(1),-map_size(2)];
stayInTheMap = constraintSafeZone(mapEdgeNormVec,mapEdgeDis2Orig,2);
basicConstraintFcn =@(xDim) {velocityConstraint(xDim),stayInTheMap};
basicConstraint = basicConstraintFcn(xDim);
constraint = combineConstraints(basicConstraint{:});
%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.05;
penalty = penalty_value;
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
fileNameSave=[mfilename regexprep(char(datetime('now')),'[: ]+','-')];
save(fileNameSave)
%% with safezone constraint
y = reshape(x,2,[],n_agent);
for i = 1:n_agent
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end
uncertainty_map = uncertaintyMap(x);
figure()
meshz(uncertainty_map)
save('base_result.mat')
x1_ref = y(:,:,1);
x2_ref = y(:,:,2);
u1_ref = diff(x1_ref,1,2);
u2_ref = diff(x2_ref,1,2);