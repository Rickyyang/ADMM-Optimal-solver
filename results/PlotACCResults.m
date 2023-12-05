close all
% Load data and setup environments
load('FinalResult.mat')
result_text = fileread('ACCSimulation.json');
result = jsondecode(result_text);
y = zeros([size(result(1).Traj),5]);
n_agent = length(result);
for i=1:n_agent
    y(:,:,i)=result(i).Traj;
end
normalVec_1 =  [1,0,-1,0;...
               0,1, 0,-1];  
dis2Origin_1 = [4,2,-3,1]';
normalVec_s =  [1,0,-1,0;... %0,-1, 0;...
               0,1, 0,-1];  %1, 0,-1];
dis2Origin_s = [4,3,-3,-2]';% 5,-4,-3];
% avoid_point = [1;5];
safeZoneNormalVec = {normalVec_1,normalVec_s};%{normalVec_1,normalVec_2};
safeZonedis2Origin = {dis2Origin_1,dis2Origin_s};%{dis2Origin_1,dis2Origin_2};

% setup figure
figure('Name','Results','Position', [10 10 900 800])
C_traj = {'k','b','r','g','y',[.1 .1 .1],[.8 .2 .6]} ;
group = [result(:).GroupID];
% Plot Start pos
for i = 0:max(group)
    group_start = reshape(y(:,1,:),2,[])*(group==i)'/sum(group==i);
    plotCircle(group_start(1),group_start(2),0.3,'LineStyle',":");
    group_end = reshape(y(:,length(y),:),2,[])*(group==i)'/sum(group==i);
% %     plotCircle(group_end(1),group_end(2),0.3,'LineStyle',":");
end
hold on
% Plot online tasks
Online_task_color={'#03719c','#789b73','#03719c','#b790d4'};
plotPoints([2;6.5],'*','MarkerFaceColor',Online_task_color{1},'MarkerSize',20,'linewidth',3,'DisplayName','Fulfilled Online Task 1');
plotPoints([-1;3],'*','MarkerFaceColor',Online_task_color{2},'MarkerSize',20,'linewidth',3,'DisplayName','fulfilled Online Task 2');
% add unfulfilled tasks
plotPoints([0;6],'x','MarkerFaceColor',Online_task_color{3},'MarkerSize',10,'linewidth',3,'DisplayName','Unfilfilled Online Task 3');
plotPoints([6;3],'x','MarkerFaceColor',Online_task_color{4},'MarkerSize',10,'linewidth',3,'DisplayName','Unfilfilled Online Task 4');

% Plot trajectories
y=y(:,1:20,:);
% col = parula(n_agent); 
col = {'#7e1e9c','#15b01a','#0343df','#653700','#e50000','#029386','#f97306','#d1b26f'};
for i = 1:n_agent
    txt = ['Group',num2str(result(i).GroupID+1),'Agent ',num2str(result(i).AgentID+1)];
    plotPoints(y(:,:,i),'-x','DisplayName',txt,'LineWidth',2,'Color', col{i});
%     plotPoints(y(:,12,i),'o','MarkerFaceColor',col(i,:),'MarkerSize',20);
     plotCircle(y(1,end,i),y(2,end,i),0.15,'FaceColor',col{i})
    hold on
end
legend
% plot obstacle & safezone
obstacle(1).normalVec=normalVec_1;
obstacle(1).dis2Origin=dis2Origin_1';
obstacle(2).normalVec=normalVec_s;
obstacle(2).dis2Origin=dis2Origin_s';
plotSafeZone(obstacle)

% plot txt
str_s = 'Start';
text(0,-0.2,['Group1 ',str_s],'FontSize',12);
text(2,-0.2,['Group2 ',str_s],'FontSize',12);
text(5,-0.2,['Group3 ',str_s],'FontSize',12);
% str_e = 'End';
% xEnd=[5 5;3 5;2 2]';
% text(4.8,5.5,['Group1 ',str_e],'FontSize',12);
% text(2.4,5.2,['Group2 ',str_e],'FontSize',12);
% text(1.7,2,['Group3 ',str_e],'FontSize',12);
axis([-1 7.5 -1 7.5])

% xx = linspace(0,6,7);
% yy = linspace(0,6,7);
% uncertainty_map = uncertaintyMap(x);

% uncertainty_map = uncertaintyMap(x);
% figure()
% meshz(uncertainty_map)