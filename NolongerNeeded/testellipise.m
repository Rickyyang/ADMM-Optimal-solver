close all
clc
clear

methodSolver='admm';

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
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
% reachability constraint
timePeriod1 = [7,13];
fix_point1 = [5;5];
ellipiseConstraint1 = constraintEllipise(timePeriod1,v_max,fix_point1,n_agent);
% timePeriod2 = [10,13];
% fix_point2 = [6;7];
% ellipiseConstraint2 = constraintEllipise(timePeriod2,v_max,fix_point2,n_agent);
% combine multiple constrains
% constraint = velocityConstraint;
constraint = combineConstraints(velocityConstraint,ellipiseConstraint1);

%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.1;
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
[x,output] = admm_solver(problem,'debug');
x = reshape(x,2,[]);
%% visualization of the solution
plotPoints([xStart,x,xEnd],'-rx');
hold on
% plotPoints([xStart,x_initial,xEnd],'-bo')
focus1 = x(:,timePeriod1);
% focus2 = x(:,timePeriod2);
a1 = diff(timePeriod1)*v_max/2;
% a2 = diff(timePeriod2)*v_max/2;
drawellipise(focus1,a1,fix_point1);
% drawellipise(focus2,a2,fix_point2);

