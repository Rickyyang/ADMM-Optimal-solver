load('unsafe_result.mat')
load('traj_and_obstacle.mat')
close all
f=figure();
y = reshape(x,2,[],n_agent);
for i = 1:n_agent
txt = ['agent ',num2str(i)];
plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-x','DisplayName',txt,'LineWidth',2);
hold on
end
legend
str_s = 'Start';
font_size=20;
text(0.1,0.3,str_s,'FontSize',font_size);
text(5,0.2,str_s,'FontSize',font_size);
text(9.3,0.2,str_s,'FontSize',font_size);
str_e = 'Goal';
xEnd=[5 5;3 5;2 2]';
text(9.3,9.8,str_e,'FontSize',font_size);
text(5,9.8,str_e,'FontSize',font_size);
text(3.7,5,str_e,'FontSize',font_size);
obstacle(1).dis2Origin = [7.8000 0 -6.2000 -4];
obstacle(2).dis2Origin = [2 8 -1 -7];
plotSafeZone(obstacle)
uncertainty_map = uncertaintyMap(x);
mesh_x = linspace(0,10,11);
mesh_y = mesh_x;
contour(mesh_x,mesh_y,uncertainty_map,'DisplayName','Field Information')
axis([0,10,0,10])
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
f.Position = [100 100 900 900];