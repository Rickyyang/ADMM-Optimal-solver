% custom solver test
close all
clc
clear

methodSolver='admm';

% load("init_wp.mat");
load("test_field.mat");
%% trajectory properities
d=100;
preH=randn(d);
H=preH'*preH;
f=randn(d,1);
%H = [1 -1;-1 2];
%f = [-2;-6];
costFcn = @(x) 0.5*x'*H*x+f'*x;
costFcnDer = @(x) 0.5*(H+H')*x + f;
xDim = [d,1];
x_initial = randn(d,1);
%% constrains
A=randn(1,d);
%A = [2,3];
b = 1;
AIneq=eye(d);
bIneq=zeros(d,1);

% velocity for general solver
constraint_eq = constraintLinearEq(A,b,xDim);
constraint_ineq = constraintLinearInEq(AIneq,bIneq,xDim);
constraint = combineConstraints(constraint_eq,constraint_ineq);
%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty_value = 1;
penalty = penalty_value*ones(vectorizedZDim,1);
penaltyUpdate = 'constant';
%% make combine all info send to admm solver

problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
problem.flagCustomSolver = false;
%% admm function call
tic
[x,output] = admm_solver(problem,'debug');
toc
tic
xqp = quadprog(H,f,-AIneq,bIneq,A,b);
toc
%% custom solver
customSolver =@(z,u,p,x) custom_solver(z,u,p,x,H,f,A,b,xDim);
constraint = constraintLinearInEq(AIneq,bIneq,xDim);
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = penalty_value*ones(vectorizedZDim,1);
problem.constraint = constraint;
problem.penalty = penalty;
problem.flagCustomSolver = true;
problem.primalSolver = customSolver;

tic
[x_custom,output] = admm_solver(problem,'debug');
toc
x
x_custom
xqp
plot(output.primal_residual)
hold on
grid on
plot(output.dual_residual)

function  solver_out =custom_solver(z,u,p,x,H,f,A,b,xDim)
d=size(H,1);
p = p(1);
lhs = [H+p*eye(d),A';A,0];
rhs = -[f-p*(z-u);-b];
result = lhs\rhs;
solver_out = result(1:xDim(1));
end