function newTraj = increase_traj_density(traj,xStart,multiple)
n_agent = size(xStart,2);
traj=reshape(traj,2,[],n_agent);
xStart = reshape(xStart,2,1,n_agent);
traj_edge = diff([xStart,traj],[],2);
traj_edge_new = traj_edge/multiple;
new_traj_length = size(traj,2)*multiple;
traj_new = zeros(size(traj,1),new_traj_length,size(traj,3));
traj_new(:,multiple:multiple:new_traj_length,:) = traj;
for i = 1:(multiple-1) 
   traj_new(:,i:multiple:new_traj_length,:) = traj - (multiple-i)*traj_edge_new;
end
newTraj = reshape(traj_new,2,[]);
end