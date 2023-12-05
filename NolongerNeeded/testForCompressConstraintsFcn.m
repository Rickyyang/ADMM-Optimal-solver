% test the compress constraint function
clc
clear
close all
%% field properity
field_prop.x = [0;2];
field_prop.Sigma = [3 0;0 3];
field_prop.A = 2;
%% trajectory properity
x_initial = [1 0;2 0.5;2 1.5;2 2.5;1.5 3.5]';
xStart = [0,0]';
xEnd = [1 4]';
n_agent = 1;
t_max=40;
r_max = 2;
velocityCap = 2;
xDim = size(x_initial);
[costFcn,costFcnDer] = trajectory_cost(field_prop);
%% constraint properities
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
% safe zone constraint
normalVec = [-1;0];
dis = -3;
safeZoneConstraint = constraintSafeZone(normalVec,dis,xDim);
safeZoneConstraint = combineConstraintsWithLogicAnd(safeZoneConstraint);
% fix point constraint
fixPoint = [4;2];
fixPointConstraint = constraintFixPoints(fixPoint,n_agent);
trajectoryConstraint = combineConstraintsWithLogicOr(safeZoneConstraint,fixPointConstraint);
% combine constraint
constraint = combineConstraints(velocityConstraint,trajectoryConstraint);
% constraint = combineConstraints(velocityConstraint,safeZoneConstraint);
% constraint = combineConstraintsWithLogicAnd(velocityConstraint);
%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.5;
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
%% make combine all info send to admm solver

problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
[x,output] = admm_solver(problem,'debug');

% plot
plotPoints([xStart,x,xEnd],'-rx');
hold on
plotPoints([xStart,x_initial,xEnd],'-bo')
legend({'Optimized trajectory','Initial trajectory'});
text(xEnd(1),xEnd(2)-0.5,'\uparrow Destination');
text(xStart(1),xStart(2)-0.5,'\uparrow Origin');

figure(2)
plot(output.primal_residual)
hold on
plot(output.dual_residual)
title('primal and dual residual')
legend('primal', 'dual')
% axis([min([xStart(1,:),xEnd(1,:)])-1 max([xStart(1,:),xEnd(1,:)])+1 min([xStart(2,:),xEnd(2,:)])-1 max([xStart(2,:),xEnd(2,:)])+1])