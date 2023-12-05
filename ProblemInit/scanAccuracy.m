function [scanAcc,scanAccDer] = scanAccuracy(traj,Sigma,map_size,map_coord,map_coord_sq)
scan_map_size = [map_size(1)+1,map_size(2)+1];
scan = zeros(scan_map_size);
scan_der = zeros(prod(scan_map_size),2);
scan(i+1,j+1)= exp(-([j;i]-traj)'/Sigma*([j;i]-traj)/2);
scan_der((j)*scan_map_size(1)+i+1,:) = (exp(-([j;i]-traj)'/Sigma*([j;i]-traj)/2)*(Sigma\([j;i]-traj)))';
scanAcc = scan(:);
scanAccDer = scan_der;
end