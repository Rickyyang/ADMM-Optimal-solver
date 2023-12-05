W_size= [6,1];
W = randn(W_size);
L = [2 -1 -1 0 0 0;-1 3 0 -1 -1 0;-1 0 2 -1 0 0;0 -1 -1 3 0 -1;0 -1 0 0 2 -1;0 0 0 -1 -1 2];
x_initial = 1/length(W)*ones(size(W));
xDim = size(x_initial);
costFcn = @(x) -x'*W;
costFcnDer = @(x) -W;
%% constrains
LinEq = constraintLinearEq(ones(size(W))',1,xDim);
LinInEq1 = constraintLinearInEq(eye(length(x_initial)),0,xDim);
%LinInEq2 = constraintLinearInEq(-eye(length(x_initial)),-1,xDim);
constraint = combineConstraints(LinEq,LinInEq1);
%% algorithm properities
%vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 1000;
penalty = 5;
penaltyUpdate = 'constant';
%% custom solver construction
problem.flagCustomSolver = false;
problem.customSolver = @(xPrev,zPrev,uPrev,p) CoeffDistributedCalculation(W,xPrev,zPrev,uPrev,p);
%% make combine all info send to admm solver
problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
[x_qp,output] = admm_solver(problem,'debug');

%% Distributed calculation
LinInEq = constraintLinearInEq([eye(length(x_initial));eye(length(x_initial))],-inf,xDim);
constraint = LinInEq;
%% custom solver construction
problem.flagCustomSolver = true;
s = 0.1;
problem.customSolver = @(xPrev,zPrev,uPrev,p) CoeffDistributedCalculation(W,L,s,xPrev,zPrev,uPrev,p);
problem.constraint = constraint;
z_initial = ones(W_size)/prod(W_size);
%problem.z_initial = [z_initial;z_initial];
%% function call
[x_d,output] = admm_solver(problem,'debug');
%linprog
f = -W;
Aeq=ones(size(W))';
beq=1;
lb=zeros(size(W));
ub=ones(size(W));
x_lp=linprog(f,[],[],Aeq,beq,lb,ub);
disp(['W:          ',num2str(W')])
disp(['lp:         ',num2str(x_lp')])
disp(['qp:         ',num2str(x_qp')])
disp(['distributed:',num2str(x_d')])

