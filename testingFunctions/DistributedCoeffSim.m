rr = 1;
nAgents=3;
nTasks=3;
Wsize=[nAgents,nTasks];
W = [1.90388697, 0.35107094, 1.25748002;
    2.71706838, 0.46700159, 1.56729068;
    0.97885058, 0.69724623, 0.77422968];
L = [2, -1,-1;-1 2 -1;-1 -1 2];
% L = [2 -1 -1 0 0 0;-1 3 0 -1 -1 0;-1 0 2 -1 0 0;0 -1 -1 3 0 -1;0 -1 0 0 2 -1;0 0 0 -1 -1 2];
K = 20;
s = 0.1;
[G,F] = MultiplierMatrix(K,L,s);
s = 0.01;
z = 1/nAgents*ones(nTasks,nAgents);
alpha = 0*W;
u = 0*z;
rho = 0.5;
maxIt = 15;
for it = 1:maxIt
    %x_update
    H = eye(nTasks);
    for i = 1:nAgents
        f = (-W(i,:)/rho-z(i,:)+u(i,:));
        Aeq = ones(1,nTasks);
        beq = 1;
        lb=zeros(nTasks,1);
%         ub=ones(nTasks,1);
        alpha_i = quadprog(H,f,[],[],Aeq,beq,lb,[]);
        alpha(i,:) = alpha_i';
    end
    for j = 1:nTasks
        z_j = G*z(:,j)+F*(alpha(:,j)+u(:,j));
        z(:,j)=z_j;
    end
    u = u+alpha-z;
end

disp('W:')
disp(W)
disp('alpha')
disp(alpha)

% Wsize = size(W);
% x_initial = 1/Wsize(1)*ones(size(W));
% x_gt = [W==max(W)]; % ground trueth
% xDim = size(x_initial);
% costFcn = @(x) -W'*x;
% costFcnDer = @(x) -W;
% %% Resular ADMM setup
% LinEq = constraintLinearEq(ones(Wsize)',1,xDim);
% LinInEq1 = constraintLinearInEq(eye(Wsize(1)),zeros(Wsize),xDim);
% LinInEq2 = constraintLinearInEq(-eye(Wsize(1)),-ones(Wsize),xDim);
% constraint = combineConstraints(LinEq,LinInEq1,LinInEq2);
% maxIt = 200;
% penalty = 1;
% penaltyUpdate = 'constant';
% problem.flagCustomSolver = false;
% problem.x_initial=x_initial;
% problem.costFcn = costFcn;
% problem.costFcnDer = costFcnDer;
% problem.constraint = constraint;
% problem.penaltyUpdate = penaltyUpdate;
% problem.penalty = penalty;
% %[x_qp,output] = admm_solver(problem,'debug');
% %% Distributed calculation
% LinInEq = constraintLinearInEq(eye(length(x_initial)),0,xDim);
% % indicate variable D(x)=z
% constraint = LinInEq;
% %% custom solver construction
% problem.flagCustomSolver = true;
% % problem.customSolver = @(xPrev,zPrev,uPrev,p) CoeffDistributedCalculation(W,L,s,xPrev,zPrev,uPrev,p,K);
% problem.customSolver = @(xPrev,zPrev,uPrev,p) MatrixUpdate(W,L,s,xPrev,zPrev,uPrev,p,K);
% problem.constraint = constraint;
% [x_d,~] = admm_solver(problem,'debug','maxit',maxIt);
% % disp(['W:          ',num2str(W')])
% % disp(['x:         ',num2str(x_d')])
% error=norm(x_d-x_gt)/prod(Wsize);

function [G,F] = MultiplierMatrix(K,L,s)
    n = size(L,1);
    F = 0*L;
    for i = 1:K
        F = F + (eye(n)-s*L)^(i-1);
    end
    F = s*F*L;
    G = (eye(n)-s*L)^K;
end

