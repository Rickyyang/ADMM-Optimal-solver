function z_out = admm_dual_update(z,r_max)
    for i = 1:length(z)
        n_zi = norm(z(:,i));
        if n_zi > r_max
            z(:,i)=z(:,i)/n_zi * r_max;
        end
    end
    z_out = z;
end