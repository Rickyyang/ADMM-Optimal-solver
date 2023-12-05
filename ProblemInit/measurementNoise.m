function scan = measurementNoise(wp,map_size)
prop.A = 1;
prop.x = wp;
prop.Sigma = [0.3,0;0,0.3];
scan = zeros(map_size);
for i = 1:map_size(1)
    for j = 1:map_size(2)
        % difference between matrix coord and actual coord
        scan(j,i) = prop.A*exp(-([i;j]-prop.x)'/prop.Sigma*([i;j]-prop.x)/2);
    end
end
end