function YOut = fieldUncertainty(traj,map_size)
traj = reshape(traj,2,[]);
Y = zeros(prod(map_size));
l = size(traj,2);
for k = 1:l
    Y = Y + scanField(traj(:,k));
end

% generate scan field for single way point
    function scan = scanField(wp)
        prop.A = 1;
        prop.x = wp;
        prop.Sigma = [0.3,0;0,0.3];
        scan = zeros(map_size);
        for i = 1:map_size(1)
            for j = 1:map_size(2)
                scan(i,j) = prop.A*exp(-([i;j]-prop.x)'/prop.Sigma*([i;j]-prop.x)/2);
            end
        end
    end
end
