function plotResults(x,problem,output,xStart,xEnd)
figure()
[xaxis,yaxis] = meshgrid(0:10);
contour(xaxis,yaxis,problem.uncertaintyMap(x),'DisplayName','Field Information')
hold on
n_agent=size(xStart,2);
y = reshape(x,2,[],n_agent);
for i = 1:n_agent
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),y(:,:,i),xEnd(:,i)],'-o','DisplayName',txt,'LineWidth',2);
end
% plotDerivative(x,problem,output)
plotSafeZone(problem.constraint.safeZones.normals,problem.constraint.safeZones.distances)
hold off
legend('show');
ylim([0,10])
xlim([0,10])

end
