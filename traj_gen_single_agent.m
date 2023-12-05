close all
clc
clear

methodSolver='admm';

% load("init_wp.mat");
load("test_field.mat");
%% trajectory properities
n_agent = 1;
xStart=[0;0];
xEnd=[10;10];
t_max=20;
v_max = 1;
velocityCap = 1;

x_initial = initTrajGen(xStart,xEnd,t_max,v_max);
xDim = size(x_initial);
[costFcn,costFcnDer] = trajectory_cost(field_prop);

%% constrains
% velocity constraint
vonstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
% localVelocityConstraint = constraintLocalVelocity(n_agent,19,velocityCap/4,[15;19],1);
% ellipise constraints
% ellipiseConstraint = constraintEllipise();
% constraint = combineConstraints(velocityConstraint,localVelocityConstraint);
% constraint = combineConstraints(velocityConstraint,safeZoneConstraint);

%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.1;
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
%% custom solver construction
problem.flagCustomSolver = false;

% problem.primalSolver = primal_solver;
%% make combine all info send to admm solver
problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
[x,output] = admm_solver(problem,'debug');

problem.x_initial = x;
constraint = combineConstraints(velocityConstraint,fixPointConstraint);
problem.constraint = constraint;
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = penalty_value*ones(vectorizedZDim,1);
problem.penalty = penalty;
[x,output] = admm_solver(problem,'debug');

%% visualization of the solution
plotPoints([xStart,x,xEnd],'-rx');
hold on
plotPoints([xStart,x_initial,xEnd],'-bo')
legend({'Optimized trajectory','Initial trajectory'});
text(xEnd(1),xEnd(2)-0.5,'\uparrow Destination');
text(xStart(1),xStart(2)-0.5,'\uparrow Origin');
text(fixPoint(1,1)+0.5,fixPoint(2,1),'\leftarrow Fix Location 1');
text(fixPoint(1,2)+0.5,fixPoint(2,2),'\leftarrow Fix Location 2');
axis([min([xStart(1,:),xEnd(1,:)])-1 max([xStart(1,:),xEnd(1,:)])+1 min([xStart(2,:),xEnd(2,:)])-1 max([xStart(2,:),xEnd(2,:)])+1])
% plot([4,4],[4,6])
% plot([6,6],[4,6])
% plot([4,6],[4,4])
% plot([4,6],[6,6])

% figure(3)
% plot(output.primal)
% title('primal')
% 
% figure(4)
% plot(output.gap)
% title('gap')
% 
figure(5)
plot(output.primal_residual)
hold on
plot(output.dual_residual)
title('primal and dual residual')
legend('primal', 'dual')
% 
% figure(6)
% plot(output.penalty(output.penalty~=0))
% title('penalty parameter')
