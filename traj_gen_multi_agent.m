% close all
% clc
% clear

% load("field_multi_agent.mat");
load("reachibility_test.mat");
%% initial trajectory generation
% load("init_wp.mat");
xStart=[0,0,;...          % starting points for all agents
        0,5,];
xEnd=[0,5;...            % Destination for all agents
      0,5];
n_agent = size(xStart,2);   % total number of agents
t_max=20;                   % maximum time limit 
traj_length = t_max-1;      % number of way-points for single agents
v_max = 1;                  % maximum speed
x_initial = initTrajGen(xStart,xEnd,t_max,v_max);   % generate initial trajectory
[costFcn,costFcnDer] = trajectory_cost(field_prop,t_max-1); 
xDim = size(x_initial);

%% constrains
% % velocity constrain
velocityCap = 1;
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
constraint = velocityConstraint;
inspection_constraints.agentPair = [1,2];
inspection_constraints.duration = 5;
inspection_constraints.meetTime = [20;20];
inspection_constraints.inspectDistance = 0.1;
inspectionConstraint=constraintInspection(inspection_constraints,traj_length,n_agent);

%% algorithm properities
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty = 0.1;
penalty = penalty*ones(vectorizedZDim,1);
penaltyUpdate = 'singleAdaptive';
%% make combine all info send to admm solver
problem.flagCustomSolver = false;
problem.x_initial=x_initial;
problem.costFcn = costFcn;
problem.costFcnDer = costFcnDer;
problem.constraint = velocityConstraint;
% problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
% general optimal value
[x_base,output] = admm_solver(problem,'debug','maxfminit',3);

% inspection constrains
constraint = combineConstraints(velocityConstraint,inspectionConstraint);
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
penalty = output.penalty(output.it,1);
penalty = penalty*ones(vectorizedZDim,1);
problem.penalty = penalty;
problem.constraint = constraint;
problem.x_initial=x_base;
% [x,output] = admm_solver(problem,'debug');

% constraint = combineConstraints(velocityConstraint,inspectionConstraint,localConstraint);
% vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
% penalty = output.penalty(output.it,1);
% penalty = penalty*ones(vectorizedZDim,1);
% problem.penalty = penalty;
% problem.constraint = constraint;
% problem.x_initial=x;
% [x,output] = admm_solver(problem,'debug');

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

% 
% %% visualization of the solution
% for i = 1:n_agent
%     wp_i_end = i*traj_length;
%     wp_i_start = (i-1)*traj_length+1;
%     x_i = x(:,wp_i_start:wp_i_end);
% %     plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-rx');
%     txt = ['agent ',num2str(i)];
%     plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-x','DisplayName',txt);
%     hold on
% end
% hold off
% legend show
% axis([min([xStart(1,:),xEnd(1,:)])-1 max([xStart(1,:),xEnd(1,:)])+1 min([xStart(2,:),xEnd(2,:)])-1 max([xStart(2,:),xEnd(2,:)])+1])
% t1 = text(xStart(1,:),xStart(2,:),'Start Point');
% t2 = text(xEnd(1,:),xEnd(2,:),'Destination');
% 
% 
% % figure(3)
% % plot(output.primal)
% % title('primal')
% % 
% % figure(4)
% % plot(output.gap)
% % title('gap')
% % 
% figure(5)
% plot(output.primal_residual)
% hold on
% plot(output.dual_residual)
% title('primal and dual residual')
% legend('primal', 'dual')

% figure(6)
% plot(output.penalty(output.penalty~=0))
% title('penalty parameter')
