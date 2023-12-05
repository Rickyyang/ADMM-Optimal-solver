%parameter for single agent case
n_agent = 1;
xStart=[0;0];
xEnd=[3;3];
t_max= 20;
r_max = 1;
velocityCap = 1;
Sigma = 0.3;
traj_length = t_max-1;

[costFcn,costFcnDer] = mapUncertaintyModel(map_size,'sigma',Sigma);
uncertaintyMap = mapUncertaintyModel(map_size,'sigma',Sigma,'uncertainty map');