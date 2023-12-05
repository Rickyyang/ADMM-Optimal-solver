clear
close all
rr = 3;
resetRands(rr)
% Wsize=[3,1];
% rr = 1;
Wsize=[6,1];
L = [2 -1 -1 0 0 0;-1 3 0 -1 -1 0;-1 0 2 -1 0 0;0 -1 -1 3 0 -1;0 -1 0 0 2 -1;0 0 0 -1 -1 2];
s = 0.1;
K = 1000;
W = 100*randn(Wsize);
x_initial = 1/Wsize(1)*ones(size(W));
x_gt = [W==max(W)]; % ground trueth
xDim = size(x_initial);
costFcn = @(x) -W'*x;
costFcnDer = @(x) -W;
%% Resular ADaaw
LinEq = constraintLinearEq(ones(Wsize)',1,xDim);
LinInEq1 = constraintLinearInEq(eye(Wsize(1)),zeros(Wsize),xDim);
LinInEq2 = constraintLinearInEq(-eye(Wsize(1)),-ones(Wsize),xDim);
constraint = combineConstraints(LinEq,LinInEq1,LinInEq2);
maxIt = 200;
penalty = 1;
penaltyUpdate = 'constant';
problem.flagCustomSolver = false;
problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%[x_qp,output] = admm_solver(problem,'debug');
%% Distributed calculation
LinInEq = constraintLinearInEq(eye(length(x_initial)),0,xDim);
% indicate variable D(x)=z
constraint = LinInEq;
%% custom solver construction
problem.flagCustomSolver = true;
% problem.customSolver = @(xPrev,zPrev,uPrev,p) CoeffDistributedCalculation(W,L,s,xPrev,zPrev,uPrev,p,K);
problem.customSolver = @(xPrev,zPrev,uPrev,p) MatrixUpdate(W,L,s,xPrev,zPrev,uPrev,p,K);
problem.constraint = constraint;
[x_d,output] = admm_solver(problem,'debug','maxit',maxIt);
A = MultiplierMatrixOutput(K,L,s);
[V,D] = eig(A);
% plot(eig(A),'o')
% A
% V
% axis equal
% x_d'
% output.uAll(:,:,output.it)'
% plot(output.lag(2:50))
figure(1)
plot(squeeze(output.xAll)')
uAll=squeeze(output.uAll);
zAll=squeeze(output.zAll);
figure(2)
plot(uAll')
figure(3)
plot(zAll')

for i = 1:output.it
yz(i) = uAll(:,i)'*zAll(:,i);
yzo(i)=uAll(:,i)'*x_d;
end
diff = yzo-yz;
figure(4)
plot(diff)
