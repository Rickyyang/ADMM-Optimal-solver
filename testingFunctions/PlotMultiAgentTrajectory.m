function PlotMultiAgentTrajectory(x)
load('agentProperities.mat')
for i = 1:n_agent
    wp_i_end = i*traj_length;
    wp_i_start = (i-1)*traj_length+1;
    x_i = x(:,wp_i_start:wp_i_end);
    %     plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-rx');
    txt = ['agent ',num2str(i)];
    plotPoints([xStart(:,i),x_i,xEnd(:,i)],'-x','DisplayName',txt);
    hold on
end


