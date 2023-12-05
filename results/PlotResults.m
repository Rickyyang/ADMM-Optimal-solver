close all
load('FinalResult.mat')
timePeriod_zone{1} = [1,4;4,16;16,traj_length]';
timePeriod_zone{2} = [1,4;4,16;16,20;20,traj_length]';
timePeriod_zone{3} = [1,8;8,16;16,traj_length]';
normalVec_1 =  [1,0,-1,0;...
               0,1, 0,-1];  
dis2Origin_1 = [4,2,-3,1]';
normalVec_s =  [1,0,-1,0;... %0,-1, 0;...
               0,1, 0,-1];  %1, 0,-1];
dis2Origin_s = [4,3,-3,-2]';% 5,-4,-3];
% avoid_point = [1;5];
safeZoneNormalVec = {normalVec_1,normalVec_s};%{normalVec_1,normalVec_2};
safeZonedis2Origin = {dis2Origin_1,dis2Origin_s};%{dis2Origin_1,dis2Origin_2};
figure(1)
figure('Name','Results','Position', [10 10 900 800])
C = {'k','b','r','g','y',[.1 .1 .1],[.8 .2 .6]} ;
col = {'#0343df','#653700','#e50000','#029386','#f97306','#d1b26f'};

for i = 1:n_agent
    txt = ['Team ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt,'LineWidth',2,'Color', col{i});
    hold on
end
legend
obstacle(1).normalVec=normalVec_1;
obstacle(1).dis2Origin=dis2Origin_1';
obstacle(2).normalVec=normalVec_s;
obstacle(2).dis2Origin=dis2Origin_s';
plotSafeZone(obstacle);
% plotSafeZone({normalVec_s},{dis2Origin_s})
% plotPoints(avoid_point,'o','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Zone 3')
x = reshape(x,2,[],n_agent);
for i = 1:n_agent
    time = timePeriod_zone{i};
    for j = 1:size(time,2)
        focus1 = x(:,time(:,j),i);
        a = diff(time(:,j))*velocityCap/2;
%         ellipseDraw(focus1,a,'-.','color',[17 17 17]/255,'HandleVisibility','off');
        plotPoints(focus1,'bo','HandleVisibility','off')
    end
end
% ellipseDraw(focus1,a,'-.','color',[17 17 17]/255,'DisplayName','Reachability Region');
% text(x(1,8,3)+0.2,x(2,8,3),'Observation','FontSize',12)
str_s = 'Start';
text(0,-0.2,str_s,'FontSize',12);
text(2,-0.2,str_s,'FontSize',12);
text(5,-0.2,str_s,'FontSize',12);
str_e = 'End';
%xEnd=[5 5;3 5;2 2]';
text(5.2,5,str_e,'FontSize',12);
text(3,5.5,str_e,'FontSize',12);
text(1.7,2,str_e,'FontSize',12);
axis([-1 7 -1 7])

xx = linspace(0,6,7);
yy = linspace(0,6,7);
uncertainty_map = uncertaintyMap(x);

% uncertainty_map = uncertaintyMap(x);
% figure()
% meshz(uncertainty_map)