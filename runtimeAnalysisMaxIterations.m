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
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
localVelocityConstraint = constraintLocalVelocity(n_agent,19,velocityCap/4,[15;19],1);
% reachability constraint
timePeriod1 = [7,13];
fix_point1 = [5;5];
ellipiseConstraint = constraintEllipise(timePeriod1,v_max,fix_point1,n_agent);
constraint = combineConstraints(velocityConstraint,localVelocityConstraint);
% constraint = combineConstraints(velocityConstraint,safeZoneConstraint);

%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 0.1;
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
%% custom solver construction
problem.flagCustomSolver = false;
%% make combine all info send to admm solver
problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
% base case
[x,output] = admm_solver(problem,'debug');
% max iteration 3

problem.x_initial = x;
constraint = combineConstraints(constraint,ellipiseConstraint);
problem.constraint = constraint;
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = penalty_value*ones(vectorizedZDim,1);
problem.penalty = penalty;

tic
[x,output] = admm_solver(problem,'debug');
num_it(1) = output.it;
primal(1) = output.primal(output.it);
primal_residual(1) = output.primal_residual(output.it);
dual_residual(1) = output.dual_residual(output.it);
T(1) = toc;
for i = 2:6
    tic
    [x,output] = admm_solver(problem,'debug','maxfminit',i-1);
    num_it(i) = output.it;
    primal(i) = output.primal(output.it);
    primal_residual(i) = output.primal_residual(output.it);
    dual_residual(i) = output.dual_residual(output.it);
    T(i) = toc;
end

% %% visualization of the solution
% 
x_axis = categorical({'RunTime','Primal','Total Iterations','Primal Residual','Dual Residual'},{'RunTime','Primal','Total Iterations','Primal Residual','Dual Residual'});
y_axis = abs([T;primal;num_it;10*primal_residual;10*dual_residual]);
bar(x_axis,y_axis);
l = cell(1,5);
l{1}='Default'; l{2}='1'; l{3}='2'; l{4}='3'; l{5}='4';
lgd = legend(l);
title(lgd,'Max Iteration')
% figure(1)
% plot(output.primal_residual)
% hold on
% plot(output.dual_residual)
% title('primal and dual residual')
% legend('primal', 'dual')
% figure(2)
% plot(output_3.primal_residual)
% hold on
% plot(output_3.dual_residual)
% title('primal and dual residual')
% legend('primal', 'dual')
% % 
% figure(6)
% plot(output.penalty(output.penalty~=0))
% title('penalty parameter')
