% parameter for two agent case
n_agent = 2;
xStart=[0 3;0 0];
xEnd=[3 0;3 3];
t_max= 20;
r_max = 1;
velocityCap = 1;
Sigma = 0.3;
traj_length = t_max-1;

[costFcn,costFcnDer] = mapUncertaintyModel(map_size,'agent number',2,'trajectory length',traj_length,'sigma',Sigma);
uncertaintyMap = mapUncertaintyModel(map_size,'agent number',2,'trajectory length',traj_length,'uncertainty map','sigma',Sigma);