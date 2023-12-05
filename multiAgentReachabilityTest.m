close all
clc
clear

load("results\reachibility_test.mat");
%% initial trajectory generation
% load("init_wp.mat");
xStart=[0,0;...          % starting points for all agents
        0,10];
xEnd=[10,10;...            % Destination for all agents
      0,10];
n_agent = size(xStart,2);   % total number of agents
t_max=20;                   % maximum time limit 
traj_length = t_max-1;      % number of way-points for single agents
v_max = 1;                  % maximum speed
% x_initial = initTrajGen(xStart,xEnd,t_max,v_max);   % generate initial trajectory
load('x_init.mat')
x_initial = x;
[costFcn,costFcnDer] = trajectory_cost(field_prop,t_max-1); 
xDim = size(x_initial);

%% constrains
% velocity constraint
velocityCap = 1;
velocityConstraint = constraintVelocity(xStart,xEnd,xDim,velocityCap);
% % reachability constraint to avoid point
% timePeriod1 = [5,10];
% fix_point1 = [3;3];
% ellipiseConstraint1 = constraintEllipise(timePeriod1,v_max,fix_point1,n_agent);
% timePeriod2 = [10,15];
% ellipiseConstraint2 = constraintEllipise(timePeriod2,v_max,fix_point1,n_agent);
% reachability constraint to avoid half-plane
meetTime = [6;14];

n = [1,0;0,1;-1,0;0,-1]';
d = [6;4;-4;0];
timePeriod_1 = [1;meetTime(1)];
zoneType = 'out';
ellipse2SafezoneConstraint{1} = constraintSafeZoneTimeSpecified(timePeriod_1,v_max,n,d,n_agent,zoneType,'agents_involved',1);
ellipse2SafezoneConstraint{2} = constraintSafeZoneTimeSpecified(timePeriod_1,v_max,n,d,n_agent,zoneType,'agents_involved',2);

timePeriod_2 = [meetTime(1);meetTime(2)];
zoneType = 'out';
ellipse2SafezoneConstraint{3} = constraintSafeZoneTimeSpecified(timePeriod_2,v_max,n,d,n_agent,zoneType);

timePeriod_3 = [meetTime(2);19];
zoneType = 'out';
ellipse2SafezoneConstraint{4} = constraintSafeZoneTimeSpecified(timePeriod_3,v_max,n,d,n_agent,zoneType);

reachability_constraint = combineConstraints(ellipse2SafezoneConstraint{:});

inspection_constraints.agentPair = [1,2];
inspection_constraints.duration = 1;
inspection_constraints.meetTime = meetTime(1);
inspection_constraints.inspectDistance = 0.1;
inspectionConstraint_1=constraintInspection(inspection_constraints,traj_length,n_agent);

inspection_constraints.meetTime = meetTime(2);
inspectionConstraint_2=constraintInspection(inspection_constraints,traj_length,n_agent);

inspectionConstraint=combineConstraints(inspectionConstraint_1,inspectionConstraint_2);
constraint = combineConstraints(velocityConstraint,inspectionConstraint);%,ellipse2SafezoneConstraint);


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
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
%% admm function call
% general optimal value
% tic
% [x,output] = admm_solver(problem,'debug','maxfminit',4);
% T(1)=toc;
% % n = [-1;0];
% % d = 1.5;
% % ellipse2planeConstraint = constraintEllipse2Halfplane(timePeriod1,v_max,n,d,n_agent);
constraint = combineConstraints(constraint,reachability_constraint);
vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
maxIt = 200;
penalty = 0.1;
penalty = penalty*ones(vectorizedZDim,1);
problem.x_initial=x;
problem.constraint = constraint;
problem.penaltyUpdate = penaltyUpdate;
problem.penalty = penalty;
tic
[x,output] = admm_solver(problem,'debug','maxfminit',4);
T(2) = toc;
%% visualization of the solution
for i = 1:n_agent
    wp_i_end = i*traj_length;
    wp_i_start = (i-1)*traj_length+1;
    x_i = x(:,wp_i_start:wp_i_end);
%     plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-rx');
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end
% hold off
x = reshape(x,2,[],n_agent);
% for i = 1:n_agent
%     focus1 = x(:,timePeriod_1,i);
%     a = diff(timePeriod_1)*v_max/2;
%     ellipseDraw(focus1,a);
%     plotPoints(focus1,'bo')
% end
% 
% for i = 1:n_agent
%     focus1 = x(:,timePeriod_2,i);
%     a = diff(timePeriod_2)*v_max/2;
%     ellipseDraw(focus1,a);
%     plotPoints(focus1,'bo')
% end
% 
% for i = 1:n_agent
%     focus1 = x(:,timePeriod_3,i);
%     a = diff(timePeriod_3)*v_max/2;
%     ellipseDraw(focus1,a);
%     plotPoints(focus1,'bo')
% end
nAlt = [n(:,end),n,n(:,1)];
dAlt = [d(end);d;d(1)];
corner = zeros(2,5);
for i = 1:length(d)+1
    corner(:,i)=nAlt(:,i:i+1)'\dAlt(i:i+1);
end
plotPoints(corner,'b-')
grid on
axis equal
hold off
% legend show
% axis([min([xStart(1,:),xEnd(1,:)])-1 max([xStart(1,:),xEnd(1,:)])+1 min([xStart(2,:),xEnd(2,:)])-1 max([xStart(2,:),xEnd(2,:)])+1])
% t1 = text(xStart(1,:),xStart(2,:),'Start Point');
% t2 = text(xEnd(1,:),xEnd(2,:),'Destination');


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

% figure(6)
% plot(output.penalty(output.penalty~=0))
% title('penalty parameter')
