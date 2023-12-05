function z_out = admm_dual_update_fixed_point(z,r_max)
    for i = 1:(length(z)-1)
        n_zi = norm(z(:,i));
        if n_zi > r_max
            z(:,i)=z(:,i)/n_zi * r_max;
        end
    end
    z(:,length(z))=[0;0];
    z_out = z;
end